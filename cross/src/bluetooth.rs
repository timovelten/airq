use defmt::{info, unwrap};
use defmt::{warn, Debug2Format};
use embassy_futures::select::select;
use embassy_futures::select::select3;
use trouble_host::prelude::*;
use trouble_host::Address;

use crate::Signal;
use crate::Signals;

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
    air_quality: AirQualityService,
}

// See: https://github.com/embassy-rs/trouble/blob/main/examples/apps/src/ble_bas_peripheral.rs

/// See
/// - Environmental sensing service https://bitbucket.org/bluetooth-SIG/public/src/main/assigned_numbers/profiles_and_services/ess/ess_permitted_characteristics.yaml
/// - Specification of characteristics (including units) (supplement) https://bitbucket.org/bluetooth-SIG/public/src/main/gss/
/// - Assigned numbers to characteristics https://bitbucket.org/bluetooth-SIG/public/src/main/assigned_numbers/uuids/characteristic_uuids.yaml
///
/// Note that we do not use the environmental sensing service profile, because they are not allowed to expose co2 and also
/// we have to implement sfloat (and that is kind of a weird format tbh)
#[gatt_service(uuid = "0e201142-fba1-43a9-b7c9-49020b000000")]
struct AirQualityService {
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

pub async fn run<C: Controller>(controller: C, signals: &'static Signals) {
    let address = Address::random([0x41, 0x5A, 0xE3, 0x1E, 0x83, 0xE8]);
    info!("Our address = {:?}", address);

    let mut resources = Resources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

    let stack = stack.build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    info!("Starting advertising and GATT service");
    let server = unwrap!(Server::new_with_config(GapConfig::Peripheral(
        PeripheralConfig {
            name: "TrouBLE",
            appearance: &appearance::UNKNOWN,
        }
    ),));

    let app_task = async {
        loop {
            signals.active_connection.signal(false);

            match advertise("Pico Air Quality", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    // run until any task ends (usually because the connection has been closed), then return to advertising state.
                    signals.active_connection.signal(true);

                    select3(
                        conn_task(&conn),
                        update_pm(&signals.pm_update, &conn, &server),
                        update_co2(&signals.co2_update, &conn, &server),
                    )
                    .await;
                }
                Err(BleHostError::BleHost(err)) => {
                    defmt::panic!("[adv] host error: {:?}", err);
                }
                Err(BleHostError::Controller(err)) => {
                    defmt::panic!("[adv] controller error: {}", Debug2Format(&err));
                }
            }
        }
    };

    let ble_task = async {
        loop {
            if let Err(BleHostError::BleHost(err)) = runner.run().await {
                defmt::panic!("[gatt_task] error: {:?}", err);
            }
        }
    };

    select(ble_task, app_task).await;
}

async fn update_pm<P: PacketPool>(
    pm_signal: &'static Signal<sps30::Measurement>,
    conn: &GattConnection<'_, '_, P>,
    server: &Server<'_>,
) {
    let service = &server.air_quality;

    loop {
        let values = pm_signal.wait().await;

        let pm1 = (values.mass_pm1 * 100.0) as u16;
        let pm2 = (values.mass_pm2 * 100.0) as u16;
        let pm4 = (values.mass_pm4 * 100.0) as u16;
        let pm10 = (values.mass_pm10 * 100.0) as u16;

        let _ = service.pm1.notify(conn, &pm1, true).await;
        let _ = service.pm2.notify(conn, &pm2, true).await;
        let _ = service.pm4.notify(conn, &pm4, true).await;
        let _ = service.pm10.notify(conn, &pm10, true).await;
    }
}

async fn update_co2<P: PacketPool>(
    co2_signal: &'static Signal<scd30::Measurement>,
    conn: &GattConnection<'_, '_, P>,
    server: &Server<'_>,
) {
    let service = &server.air_quality;

    loop {
        let values = co2_signal.wait().await;

        let co2 = values.co2_concentration as u16;
        let temp = (values.temperature * 100.0) as i16;
        let rh = (values.relative_humidity * 100.0) as u16;

        let _ = service.co2.notify(&conn, &co2, true).await;
        let _ = service.relative_humidity.notify(&conn, &rh, true).await;
        let _ = service.temperature.notify(&conn, &temp, true).await;
    }
}

async fn conn_task<P: PacketPool>(conn: &GattConnection<'_, '_, P>) {
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                // We could inspect the event here and then maybe change our behavior
                // based on what values are read (?), but we don't need to.

                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(err) => warn!("Error sending response: {:?}", err),
                }
            }
            _ => {}
        }
    }

    info!("[gatt] task finished");
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
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
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");

    Ok(conn)
}
