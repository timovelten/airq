use bt_hci::controller::ExternalController;
use cyw43::bluetooth::BtDriver;
use defmt::debug;
use defmt::info;
use defmt::Debug2Format;
use embassy_futures::select::select;
use embassy_futures::select::select3;
use trouble_host::prelude::*;
use trouble_host::Address;

use crate::Signal;

/// Size of L2CAP packets (ATT MTU is this - 4)
const L2CAP_MTU: usize = 251;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

type Resources<C> = HostResources<C, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU>;

// GATT Server definition
#[gatt_server]
struct Server {
    custom: CustomService,
}

/// See
/// - Environmental sensing service https://bitbucket.org/bluetooth-SIG/public/src/main/assigned_numbers/profiles_and_services/ess/ess_permitted_characteristics.yaml
/// - Specification of characteristics (including units) (supplement) https://bitbucket.org/bluetooth-SIG/public/src/main/gss/
/// - Assigned numbers to characteristics https://bitbucket.org/bluetooth-SIG/public/src/main/assigned_numbers/uuids/characteristic_uuids.yaml
///
/// Note that we do not use the environmental sensing service profile, because they are not allowed to expose co2 and also
/// we have to implement sfloat (and that is kind of a weird format tbh)
#[gatt_service(uuid = "0e201142-fba1-43a9-b7c9-49020b000000")]
struct CustomService {
    /// org.bluetooth.characteristic.temperature,
    ///
    /// Celcius, where 2150 is 21.50 degree celcius
    #[characteristic(uuid = "2A6E", read, notify)]
    temperature: i16,

    // org.bluetooth.characteristic.humidity
    //
    // Percent, where 4567 is 45.67 percent relative humidity
    #[characteristic(uuid = "2A6F", read, notify)]
    relative_humidity: u16,

    /// org.bluetooth.characteristic.co2_concentration
    ///
    /// ppm
    #[characteristic(uuid = "2B8C", read, notify)]
    co2: u16,

    /// Custom, same encoding as temperature and humidity, i.e. two decimal places
    #[characteristic(uuid = "0e201142-fba1-43a9-b7c9-49020b000001", read, notify)]
    pm1: u16,

    /// Custom, same encoding as temperature and humidity, i.e. two decimal places
    #[characteristic(uuid = "0e201142-fba1-43a9-b7c9-49020b000002", read, notify)]
    pm2: u16,

    /// Custom, same encoding as temperature and humidity, i.e. two decimal places
    #[characteristic(uuid = "0e201142-fba1-43a9-b7c9-49020b000004", read, notify)]
    pm4: u16,

    /// Custom, same encoding as temperature and humidity, i.e. two decimal places
    #[characteristic(uuid = "0e201142-fba1-43a9-b7c9-49020b000010", read, notify)]
    pm10: u16,
}

#[embassy_executor::task]
pub async fn runner(
    bt_device: BtDriver<'static>,
    pm_signal: &'static Signal<sps30::Measurement>,
    co2_signal: &'static Signal<scd30::Measurement>,
) {
    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    let address = Address::random([0x41, 0x5A, 0xE3, 0x1E, 0x83, 0xE8]);
    info!("Our address = {:?}", address);

    let mut resources = Resources::new(PacketQos::None);
    let (stack, mut peripheral, _, runner) = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(
        stack,
        GapConfig::Peripheral(PeripheralConfig {
            name: "TrouBLE",
            appearance: &appearance::UNKNOWN,
        }),
    )
    .unwrap();

    let ble_background_tasks = select(ble_task(runner), gatt_task(&server));
    let app_task = async {
        loop {
            match advertise("Pico Air Quality", &mut peripheral).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    // run until any task ends (usually because the connection has been closed), then return to advertising state.
                    select3(
                        wait_for_connection_end(&conn),
                        update_pm(pm_signal, &server, &conn),
                        update_co2(co2_signal, &server, &conn),
                    )
                    .await;
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", Debug2Format(&e));
                }
            }
        }
    };
    select(ble_background_tasks, app_task).await;
}

async fn update_pm<C: Controller>(
    pm_signal: &'static Signal<sps30::Measurement>,
    server: &Server<'_, '_, C>,
    conn: &Connection<'_>,
) {
    let service = &server.custom;

    loop {
        let values = pm_signal.wait().await;

        let pm1 = (values.mass_pm1 * 100.0) as u16;
        let pm2 = (values.mass_pm2 * 100.0) as u16;
        let pm4 = (values.mass_pm4 * 100.0) as u16;
        let pm10 = (values.mass_pm10 * 100.0) as u16;

        let r1 = server.notify(&service.pm1, &conn, &pm1).await;
        let r2 = server.notify(&service.pm2, &conn, &pm2).await;
        let r3 = server.notify(&service.pm4, &conn, &pm4).await;
        let r4 = server.notify(&service.pm10, &conn, &pm10).await;

        let failures = (r1.is_ok(), r2.is_ok(), r3.is_ok(), r4.is_ok());
        debug!("Updated PM {}", failures);
    }
}

async fn update_co2<C: Controller>(
    co2_signal: &'static Signal<scd30::Measurement>,
    server: &Server<'_, '_, C>,
    conn: &Connection<'_>,
) {
    let service = &server.custom;

    loop {
        let values = co2_signal.wait().await;

        let co2 = values.co2_concentration as u16;
        let temp = (values.temperature * 100.0) as i16;
        let rh = (values.relative_humidity * 100.0) as u16;

        let _ = server.notify(&service.co2, &conn, &co2).await;
        let _ = server.notify(&service.relative_humidity, &conn, &rh).await;
        let _ = server.notify(&service.temperature, &conn, &temp).await;
    }
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", Debug2Format(&e));
        }
    }
}

async fn gatt_task<C: Controller>(server: &Server<'_, '_, C>) {
    loop {
        if let Err(e) = server.run().await {
            panic!("[gatt_task] error: {:?}", Debug2Format(&e));
        }
    }
}

async fn wait_for_connection_end(conn: &Connection<'_>) {
    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                break;
            },
            _ => {}
        }
    }

    info!("[gatt] task finished");
}

async fn advertise<'a, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C>,
) -> Result<Connection<'a>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?;
    info!("[adv] connection established");
    Ok(conn)
}
