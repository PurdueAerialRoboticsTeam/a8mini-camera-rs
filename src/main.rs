use std::io::{self, Write};

use a8mini_camera_rs::control::{
    A8MiniComplexCommand, A8MiniComplexHTTPQuery, A8MiniSimpleCommand, A8MiniSimpleHTTPQuery,
    A8MiniAttitude,
};
use a8mini_camera_rs::A8Mini;
use chrono::Utc;
use tokio::fs::{File, OpenOptions};
use tokio::io::AsyncWriteExt;
use tracing::Level;
use bincode::deserialize;

fn print_ascii_command_table() {
    let simple_commands = [
        "AutoCenter",
        "RotateUp",
        "RotateDown",
        "RotateRight",
        "RotateLeft",
        "StopRotation",
        "ZoomIn",
        "ZoomOut",
        "ZoomMax",
        "MaxZoomInformation",
        "FocusIn",
        "FocusOut",
        "TakePicture",
        "RecordVideo",
        "Rotate100100",
        "CameraInformation",
        "AutoFocus",
        "HardwareIDInformation",
        "FirmwareVersionInformation",
        "SetLockMode",
        "SetFollowMode",
        "SetFPVMode",
        "AttitudeInformation",
        "SetVideoOutputHDMI",
        "SetVideoOutputCVBS",
        "SetVideoOutputOff",
        "LaserRangefinderInformation",
        "RebootCamera",
        "RebootGimbal",
        "Resolution4k",
        "Heartbeat",
        "GimbalStatus",
    ];

    let complex_commands = [
        "SetYawPitchSpeed(i8, i8)",
        "SetYawPitchAngle(i16, i16)",
        "SetTimeUTC(u64)",
        "GetCodecSpecs(u8)",
        "SetCodecSpecs(u8, u8, u16, u16, u16, u8)",
        "LogAttitudeStream", 
    ];

    let simple_queries = [
        "GetDirectoriesPhotos",
        "GetDirectoriesVideos",
        "GetMediaCountPhotos",
        "GetMediaCountVideos",
    ];

    let complex_queries = [
        "GetPhoto(u32)",
        "GetVideo(u32)"
    ];

    let all_printed = [
        simple_commands.to_vec(),
        complex_commands.to_vec(),
        simple_queries.to_vec(),
        complex_queries.to_vec(),
    ];

    for list in all_printed.iter() {
        let bar = "+----+------------------------------+";
        println!("{}", bar);
        println!("| ID | Command Name                 |");
        println!("{}", bar);

        for (i, item) in list.iter().enumerate() {
            println!("| {:>2} | {:<28} |", i, item);
        }

        println!("{}", bar);
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(Level::DEBUG)
        .init();

    print_ascii_command_table();

    loop {
        let full_command: &str;
        println!("Awaiting command: ");
        let stdin = io::stdin();
        let buf = &mut String::new();
        stdin.read_line(buf)?;
        full_command = buf.strip_suffix("\n").unwrap();

        let destructured_command: Vec<&str> = full_command.split(" ").collect();
        let command: &str = destructured_command[0];

        // GIMBAL ATTITUDE INFORMATION LOGGING TEST WITH 100 HZ
        if command == "LogAttitudeStream" {
            println!("Starting 100Hz Active Polling Log...");
            let camera = A8Mini::connect().await?;

            let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S").to_string();
            let filename = format!("attitude_log_{}.csv", timestamp);
            
            let mut file = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&filename)
                .await?;

            file.write_all(b"Timestamp,Yaw,Pitch,Roll,V_Yaw,V_Pitch,V_Roll\n").await?;
            println!("Logging to: {}", filename);
            println!("Polling Attitude (0x0D) at ~100Hz... (Press Ctrl+C to stop)");

            let mut buffer = [0u8; 128];
            let mut print_counter: u64 = 0; // Counter to slow down prints

            let max_failure_threshold = 10;
            let mut failure_count = 0;
            
            loop {
                // A. ACTIVELY ASK for data (Poll)
                // We use send_command_blind because we are about to listen manually immediately after
                camera.send_command_blind(A8MiniSimpleCommand::AttitudeInformation).await?;

                // B. Listen for the response with a timeout
                // We wrap this in a timeout so the loop doesn't hang forever if a packet drops
                let recv_content = tokio::time::timeout(
                    std::time::Duration::from_millis(50),
                    camera.command_socket.recv_from(&mut buffer),
                ).await.or_else(|e| {
                    // Propagate error through, but also print out vitals:
                    eprintln!("Did not get a response before 50ms timeout. Is the camera connected?");
                    Err(e)
                })?;
                
                match recv_content {
                    Ok((len, _)) => {
                        // Check if it is the correct packet (Attitude ID: 0x0D / 13)
                        if len >= 20 && buffer[7] == 0x0D {
                            let data_slice = &buffer[8..20];
                            
                            if let Ok(attitude) = deserialize::<A8MiniAttitude>(data_slice) {
                                let yaw = attitude.theta_yaw as f32 / 10.0;
                                let pitch = attitude.theta_pitch as f32 / 10.0;
                                let roll = attitude.theta_roll as f32 / 10.0;
                                
                                // 1. ALWAYS Log to file (Every single packet)
                                let log_line = format!(
                                    "{},{},{},{},{},{},{}\n",
                                    Utc::now().to_rfc3339(),
                                    yaw, pitch, roll,
                                    attitude.v_yaw, attitude.v_pitch, attitude.v_roll
                                );
                                file.write_all(log_line.as_bytes()).await?;

                                // 2. ONLY Print to console every 10th packet (~10Hz update rate)
                                if print_counter % 10 == 0 {
                                    print!("\rAttitude: Y: {:>6.1} | P: {:>6.1} | R: {:>6.1}", yaw, pitch, roll);
                                    io::stdout().flush().unwrap();
                                }
                                print_counter += 1;
                            }
                        }
                    },
                    Err(e) => {
                        failure_count += 1;
                        eprintln!("Failed to receive attitude from camera ({} fails): {}", failure_count, e);
                        if failure_count >= max_failure_threshold {
                            eprintln!("Failure count exceeds threshold {}. Exiting...", max_failure_threshold);
                            break;
                        }

                        continue;
                    }
                }

                // C. Throttle the loop to target ~100Hz
                // 10ms delay = 100 times per second
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }

            continue;
        }
        // end of logging block

        let simple_command_enum: Option<A8MiniSimpleCommand> = match command {
            "AutoCenter" => Some(A8MiniSimpleCommand::AutoCenter),
            "RotateUp" => Some(A8MiniSimpleCommand::RotateUp),
            "RotateDown" => Some(A8MiniSimpleCommand::RotateDown),
            "RotateRight" => Some(A8MiniSimpleCommand::RotateRight),
            "RotateLeft" => Some(A8MiniSimpleCommand::RotateLeft),
            "StopRotation" => Some(A8MiniSimpleCommand::StopRotation),
            "ZoomIn" => Some(A8MiniSimpleCommand::ZoomIn),
            "ZoomOut" => Some(A8MiniSimpleCommand::ZoomOut),
            "ZoomMax" => Some(A8MiniSimpleCommand::ZoomMax),
            "MaxZoomInformation" => Some(A8MiniSimpleCommand::MaxZoomInformation),
            "FocusIn" => Some(A8MiniSimpleCommand::FocusIn),
            "FocusOut" => Some(A8MiniSimpleCommand::FocusOut),
            "TakePicture" => Some(A8MiniSimpleCommand::TakePicture),
            "RecordVideo" => Some(A8MiniSimpleCommand::RecordVideo),
            "Rotate100100" => Some(A8MiniSimpleCommand::Rotate100100),
            "CameraInformation" => Some(A8MiniSimpleCommand::CameraInformation),
            "AutoFocus" => Some(A8MiniSimpleCommand::AutoFocus),
            "HardwareIDInformation" => Some(A8MiniSimpleCommand::HardwareIDInformation),
            "FirmwareVersionInformation" => Some(A8MiniSimpleCommand::FirmwareVersionInformation),
            "SetLockMode" => Some(A8MiniSimpleCommand::SetLockMode),
            "SetFollowMode" => Some(A8MiniSimpleCommand::SetFollowMode),
            "SetFPVMode" => Some(A8MiniSimpleCommand::SetFPVMode),
            "AttitudeInformation" => Some(A8MiniSimpleCommand::AttitudeInformation),
            "SetVideoOutputHDMI" => Some(A8MiniSimpleCommand::SetVideoOutputHDMI),
            "SetVideoOutputCVBS" => Some(A8MiniSimpleCommand::SetVideoOutputCVBS),
            "SetVideoOutputOff" => Some(A8MiniSimpleCommand::SetVideoOutputOff),
            "LaserRangefinderInformation" => Some(A8MiniSimpleCommand::LaserRangefinderInformation),
            "RebootCamera" => Some(A8MiniSimpleCommand::RebootCamera),
            "RebootGimbal" => Some(A8MiniSimpleCommand::RebootGimbal),
            "Resolution4k" => Some(A8MiniSimpleCommand::Resolution4k),
            "Heartbeat" => Some(A8MiniSimpleCommand::Heartbeat),
            "GimbalStatus" => Some(A8MiniSimpleCommand::GimbalStatus),
            _ => None,
        };

        if let Some(cmd) = simple_command_enum {
            println!("Sending Simple Command {:?}", cmd);
            let camera: A8Mini = A8Mini::connect().await?;

            match cmd {
                A8MiniSimpleCommand::AttitudeInformation => {
                    match camera.get_attitude_information().await {
                        Ok(info) => println!("{}", info), 
                        Err(e) => println!("Failed to get attitude: {:?}", e),
                    }
                },

                A8MiniSimpleCommand::FirmwareVersionInformation => {
                    match camera.get_firmware_version().await {
                        Ok(info) => println!("{}", info),
                        Err(e) => println!("Failed to get firmware version: {:?}", e),
                    }
                },

                _ => {
                    if let Ok(response) = camera.send_command(cmd).await {
                        println!("Received Response {:?}", response);
                    } else {
                        println!("Failed to receive response");
                    }
                },
            };

            continue;
        }

        let complex_command_enum: Option<A8MiniComplexCommand> = match command {
            "SetYawPitchSpeed" => {
                let yaw: i8 = destructured_command[1].parse().unwrap_or(0);
                let pitch: i8 = destructured_command[2].parse().unwrap_or(0);
                Some(A8MiniComplexCommand::SetYawPitchSpeed(yaw, pitch))
            }
            "SetYawPitchAngle" => {
                let yaw: i16 = destructured_command[1].parse().unwrap_or(0);
                let pitch: i16 = destructured_command[2].parse().unwrap_or(0);
                Some(A8MiniComplexCommand::SetYawPitchAngle(yaw, pitch))
            }
            "SetTimeUTC" => {
                let epoch: u64 = destructured_command[1].parse().unwrap_or(0);
                Some(A8MiniComplexCommand::SetTimeUTC(epoch))
            }
            "GetCodecSpecs" => {
                let stream_type: u8 = destructured_command[1].parse().unwrap_or(0);
                Some(A8MiniComplexCommand::GetCodecSpecs(stream_type))
            }
            "SetCodecSpecs" => {
                let stream_type: u8 = destructured_command[1].parse().unwrap_or(0);
                Some(A8MiniComplexCommand::SetCodecSpecs(
                    stream_type,
                    2,
                    3840,
                    2160,
                    50000,
                    0,
                ))
            }
            _ => None,
        };

        if complex_command_enum.is_some() {
            println!(
                "Sending Complex Command {:?}",
                complex_command_enum.unwrap()
            );
            let camera: A8Mini = A8Mini::connect().await?;
            camera
                .send_command_blind(complex_command_enum.unwrap())
                .await?;
            continue;
        }

        let simple_query_enum: Option<A8MiniSimpleHTTPQuery> = match command {
            "GetDirectoriesPhotos" => Some(A8MiniSimpleHTTPQuery::GetDirectoriesPhotos),
            "GetDirectoriesVideos" => Some(A8MiniSimpleHTTPQuery::GetDirectoriesVideos),
            "GetMediaCountPhotos" => Some(A8MiniSimpleHTTPQuery::GetMediaCountPhotos),
            "GetMediaCountVideos" => Some(A8MiniSimpleHTTPQuery::GetMediaCountVideos),
            _ => None,
        };

        if simple_query_enum.is_some() {
            println!("Sending Simple HTTP Query {:?}", simple_query_enum.unwrap());
            let camera: A8Mini = A8Mini::connect().await?;
            let response = camera.send_http_query(simple_query_enum.unwrap()).await?;
            println!("{:?}", response);
            continue;
        }

        let complex_query_enum: Option<A8MiniComplexHTTPQuery> = match command {
            "GetPhoto" => {
                let photo_ind: u32 = destructured_command[1].parse().unwrap_or(0);
                Some(A8MiniComplexHTTPQuery::GetPhoto(photo_ind as u32))
            }
            "GetVideo" => {
                let video_ind: u32 = destructured_command[1].parse().unwrap_or(0);
                Some(A8MiniComplexHTTPQuery::GetVideo(video_ind as u32))
            }
            _ => None,
        };

        if complex_query_enum.is_some() {
            let complex_query = complex_query_enum.unwrap();
            println!("Sending Complex HTTP Query {:?}", complex_query);
            let camera: A8Mini = A8Mini::connect().await?;

            match complex_query {
                A8MiniComplexHTTPQuery::GetPhoto(_) => {
                    let image_bytes = camera.send_http_media_query(complex_query).await?;

                    let dir = "./tmp";
                    let timestamp = Utc::now().timestamp_millis();
                    let image_path = format!("{}/IMG-{}.jpeg", dir, timestamp);

                    File::create(&image_path)
                        .await?
                        .write_all(&image_bytes)
                        .await?;
                }
                A8MiniComplexHTTPQuery::GetVideo(_) => {
                    let video_bytes = camera.send_http_media_query(complex_query).await?;

                    let dir = "./tmp";
                    let timestamp = Utc::now().timestamp_millis();
                    let vid_path = format!("{}/VID-{}.mp4", dir, timestamp);

                    File::create(&vid_path)
                        .await?
                        .write_all(&video_bytes)
                        .await?;
                } // _ => {
                  //   let response = camera.send_http_query(complex_query).await?;
                  //   println!("{:?}", response);
                  // }
            };

            continue;
        }
    }
}