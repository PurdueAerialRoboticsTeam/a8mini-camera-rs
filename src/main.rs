use std::io;

use a8mini_camera_rs::control::{A8MiniComplexCommand, A8MiniSimpleCommand, A8MiniSimpleHTTPQuery, A8MiniComplexHTTPQuery};
use a8mini_camera_rs::A8Mini;
use chrono::Utc;
use tokio::fs::File;
use tokio::io::AsyncWriteExt;


fn print_ascii_command_table() {
  let simple_commands = [
    "AutoCenter", "RotateUp", "RotateDown", "RotateRight", "RotateLeft", "StopRotation", "ZoomIn", "ZoomOut", "ZoomMax",
    "MaxZoomInformation", "FocusIn", "FocusOut", "TakePicture", "RecordVideo", "Rotate100100", "CameraInformation",
    "AutoFocus", "HardwareIDInformation", "FirmwareVersionInformation", "SetLockMode", "SetFollowMode", "SetFPVMode",
    "AttitudeInformation", "SetVideoOutputHDMI", "SetVideoOutputCVBS", "SetVideoOutputOff", "LaserRangefinderInformation", 
    "RebootCamera", "RebootGimbal", "Resolution4k", "Heartbeat"
  ];

  let complex_commands = [
    "SetYawPitchSpeed(i8, i8)",
    "SetYawPitchAngle(i16, i16)",
    "SetTimeUTC(u64)",
    "GetCodecSpecs(u8)",
    "SetCodecSpecs(u8, u8, u16, u16, u16, u8)",
  ];

  let simple_queries = [
    "GetDirectoriesPhotos",
    "GetDirectoriesVideos",
    "GetMediaCountPhotos",
    "GetMediaCountVideos",
  ];

  let complex_queries = [
    "GetPhoto(u32)",
    "GetVideo(u32)",
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
  print_ascii_command_table();
  
  loop {
    let full_command: &str;
    println!("Awaiting command:");
    let stdin = io::stdin();
    let buf = &mut String::new();
    stdin.read_line(buf)?;
    full_command = buf.strip_suffix("\n").unwrap();

    let destructured_command: Vec<&str> = full_command.split(" ").collect();
    let command: &str = destructured_command[0];

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
      _ => None,
    };

    if simple_command_enum.is_some() {
      println!("Sending Simple Command {:?}", simple_command_enum.unwrap());
      let camera: A8Mini = A8Mini::connect().await?;
      camera.send_command_blind(simple_command_enum.unwrap()).await?;
      continue;
    }

    let complex_command_enum: Option<A8MiniComplexCommand> = match command {
      "SetYawPitchSpeed" => {
        let yaw: i8 = destructured_command[1].parse().unwrap_or(0);
        let pitch: i8 = destructured_command[2].parse().unwrap_or(0);
        Some(A8MiniComplexCommand::SetYawPitchSpeed(yaw, pitch))
      },
      "SetYawPitchAngle" => {
        let yaw: i16 = destructured_command[1].parse().unwrap_or(0);
        let pitch: i16 = destructured_command[2].parse().unwrap_or(0);
        Some(A8MiniComplexCommand::SetYawPitchAngle(yaw, pitch))
      },
      "SetTimeUTC" => {
        let epoch: u64 = destructured_command[1].parse().unwrap_or(0);
        Some(A8MiniComplexCommand::SetTimeUTC(epoch))
      },
      "GetCodecSpecs" => {
        let stream_type: u8 = destructured_command[1].parse().unwrap_or(0);
        Some(A8MiniComplexCommand::GetCodecSpecs(stream_type))
      },
      "SetCodecSpecs" => {
        let stream_type: u8 = destructured_command[1].parse().unwrap_or(0);
        Some(A8MiniComplexCommand::SetCodecSpecs(stream_type, 2, 3840, 2160, 50000, 0))
      },
      _ => None,
    };

    if complex_command_enum.is_some() {
      println!("Sending Complex Command {:?}", complex_command_enum.unwrap());
      let camera: A8Mini = A8Mini::connect().await?;
      camera.send_command_blind(complex_command_enum.unwrap()).await?;
      continue;
    }

    let simple_query_enum: Option<A8MiniSimpleHTTPQuery> = match command {
      "GetDirectoriesPhotos" => Some(A8MiniSimpleHTTPQuery::GetDirectoriesPhotos),
      "GetDirectoriesVideos" => Some(A8MiniSimpleHTTPQuery::GetDirectoriesVideos),
      "GetMediaCountPhotos"  => Some(A8MiniSimpleHTTPQuery::GetMediaCountPhotos),
      "GetMediaCountVideos"  => Some(A8MiniSimpleHTTPQuery::GetMediaCountVideos),
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
        },
        A8MiniComplexHTTPQuery::GetVideo(_) => {
          let video_bytes = camera.send_http_media_query(complex_query).await?;

          let dir = "./tmp";
          let timestamp = Utc::now().timestamp_millis();
          let vid_path = format!("{}/VID-{}.mp4", dir, timestamp);

          File::create(&vid_path)
              .await?
              .write_all(&video_bytes)
              .await?;
        },
        // _ => {
        //   let response = camera.send_http_query(complex_query).await?;
        //   println!("{:?}", response);
        // }
      };
      
      continue;
    }
  }
}
