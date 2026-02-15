#![allow(non_snake_case)]

use anyhow::anyhow;
use bincode::deserialize;
use tokio::{net::UdpSocket, time::timeout};
use tracing::{debug, error, info};
use tokio::sync::mpsc;

pub mod checksum;
pub mod constants;
pub mod control;

#[derive(Debug)]
/// Represents the A8Mini camera API with a dedicate UDP socket for both `Command`s and `HTTPQuery`s.
pub struct A8Mini {
    pub command_socket: UdpSocket,
    pub http_socket: UdpSocket,
}

impl A8Mini {
    /// Connect to and creates a new `A8Mini` using default ip address `192.168.144.25` and default port 37260 and port 82. 
    /// Remote ports are mapped to port 8080 and port 8088 on local.
    pub async fn connect() -> anyhow::Result<Self> {
        Ok(Self::connect_to(
            constants::CAMERA_IP,
            constants::CAMERA_COMMAND_PORT,
            constants::CAMERA_HTTP_PORT,
            "8080",
            "8088",
        )
        .await?)
    }

    /// Repeatedly tries to reconnect a total of `max_iter`` times
    pub async fn connect_yapping(
        max_iter: i32,
    ) -> anyhow::Result<Self> {
        for _ in 1..max_iter {
            let connect_attempt = Self::connect().await;
            if connect_attempt.is_ok() {
                return Ok(connect_attempt.unwrap());
            }
        }

        Err(anyhow!("max_iter reached".to_string()))
    }

    /// Connects to and creates a new `A8Mini` given network args.
    pub async fn connect_to(
        camera_ip: &str,
        camera_command_port: &str,
        camera_http_port: &str,
        local_command_port: &str,
        local_http_port: &str,
    ) -> anyhow::Result<Self> {
        debug!(
            "Binding command_socket to {} and http_socket to {}.",
            format!("0.0.0.0:{}", local_command_port),
            format!("0.0.0.0:{}", local_http_port)
        );

        let camera: A8Mini = A8Mini {
            command_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_command_port)).await?,
            http_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_http_port)).await?,
        };

        camera
            .command_socket
            .connect(format!("{}:{}", camera_ip, camera_command_port))
            .await?;
        info!("Connected a8mini command_socket.");

        camera
            .http_socket
            .connect(format!("{}:{}", camera_ip, camera_http_port))
            .await?;
        info!("Connected a8mini http_socket.");

        Ok(camera)
    }

    /// Sends a `control::Command` blind. This should be used for all commands that don't have a ACK.
    pub async fn send_command_blind<T: control::Command>(
        &self,
        command: T,
    ) -> anyhow::Result<()> {
       

        let send_len = self.command_socket.send(command.to_bytes().as_slice()).await?;

        if send_len == 0 {
            error!("No command bytes sent.");
            return Err(anyhow!("No command bytes sent.".to_string()));
        }



        Ok(())
    }

    /// Sends a `control::Command` expecting an ACK. Returns received ACK response bytes.
    pub async fn send_command<T: control::Command>(
        &self,
        command: T,
    ) -> anyhow::Result<[u8; constants::RECV_BUFF_SIZE]> {
        self.send_command_blind(command).await?;
        let mut recv_buffer = [0; constants::RECV_BUFF_SIZE];

        debug!("Waiting for command response.");

        let recv_len = timeout(
            constants::RECV_TIMEOUT,
            self.command_socket.recv(&mut recv_buffer),
        )
        .await??;
        if recv_len == 0 {
            error!("No command bytes received.");
            return Err(anyhow!("No bytes received.".to_string()));
        }

        debug!(
            "Command response of size {} received successfully: {:?}",
            recv_len, 
            recv_buffer
        );

        Ok(recv_buffer)
    }

    /// Retrieves attitude information from the camera. 
    pub async fn get_attitude_information(
        &self,
    ) -> anyhow::Result<control::A8MiniAttitude> {
        let response = self
            .send_command(control::A8MiniSimpleCommand::AttitudeInformation)
            .await?;

        
        if response.len() < 20 {
             return Err(anyhow::anyhow!("Response too short to contain attitude data"));
        }
        
        let data_slice = &response[8..20];
        let attitude_info: control::A8MiniAttitude = deserialize(data_slice)?;
        
        Ok(attitude_info)
    }

    pub fn stream_attitude_data(self, target_hz: u64) -> mpsc::Receiver<control::A8MiniAttitude> {
        // Create a channel with a buffer of 100 packets
        let (tx, rx) = mpsc::channel(100);
        
        // Calculate sleep time (e.g., 100Hz = 10ms)
        let interval_ms = 1000 / target_hz;

        tokio::spawn(async move {
            let mut buffer = [0u8; 128];
            
            loop {
                if let Err(_) = self.send_command_blind(control::A8MiniSimpleCommand::AttitudeInformation).await {
                }

                let recv_future = self.command_socket.recv_from(&mut buffer);
                match timeout(std::time::Duration::from_millis(50), recv_future).await {
                    Ok(Ok((len, _))) => {
                        // Check for correct Packet ID (0x0D)
                        if len >= 20 && buffer[7] == 0x0D {
                            let data_slice = &buffer[8..20];
                            if let Ok(att) = deserialize::<control::A8MiniAttitude>(data_slice) {
                                
                                if tx.send(att).await.is_err() {
                                    break; 
                                }
                            }
                        }
                    },
                    _ => {}
                }

                // maintain Frequency
                tokio::time::sleep(std::time::Duration::from_millis(interval_ms)).await;
            }
        });

        rx
    }

    pub async fn get_firmware_version(&self) -> anyhow::Result<control::A8MiniFirmwareVersion> {
        let response = self
            .send_command(control::A8MiniSimpleCommand::FirmwareVersionInformation)
            .await?;

        // Header is 8 bytes. Payload is 8 bytes. Total packet should be at least 16 bytes.
        if response.len() < 16 {
             return Err(anyhow::anyhow!("Response too short for firmware version"));
        }
        
        // Slice the payload (Indices 8 to 16)
        let data_slice = &response[8..16];
        let version_info: control::A8MiniFirmwareVersion = deserialize(data_slice)?;
        
        Ok(version_info)
    }

    /// Sends a `control::HTTPQuery` and returns the corresponding received `control::HTTPResponse`.
    pub async fn send_http_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> anyhow::Result<control::HTTPResponse> {
        let response = reqwest::get(query.to_string()).await?;
        debug!("Waiting for HTTP response.");

        let json = response.json::<control::HTTPResponse>().await?;
        debug!("Received HTTP response.");
        Ok(json)
    }

    

    /// Retrieves an image or video (WIP) from the camera.
    pub async fn send_http_media_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> anyhow::Result<Vec<u8>> {
        let response = reqwest::get(query.to_string()).await?;
        info!("Waiting for HTTP response.");

        let image_bytes = response.bytes().await?;
        info!("Received HTTP response.");
        Ok(image_bytes.to_vec())
    }

    
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::thread::sleep;
    use std::time::Duration;
    use tokio::fs::File;
    use tokio::io::AsyncWriteExt;


    #[ignore]
    #[tokio::test]
    async fn test_take_and_download_photo() -> anyhow::Result<()> {
        let cam: A8Mini = A8Mini::connect().await?;

        cam.send_command(control::A8MiniSimpleCommand::TakePicture)
            .await?;
        sleep(Duration::from_millis(500));
        let num_pictures = cam
            .send_http_query(control::A8MiniSimpleHTTPQuery::GetMediaCountPhotos)
            .await?
            .data
            .count
            .unwrap();
        let picture_bytes = cam
            .send_http_media_query(control::A8MiniComplexHTTPQuery::GetPhoto(num_pictures as u32))
            .await?;
        File::create("tmp.jpeg")
            .await?
            .write_all(&picture_bytes)
            .await?;

        Ok(())
    }

    #[ignore]
    #[tokio::test]
    async fn test_send_simple_commands_blind() -> anyhow::Result<()> {
        let cam: A8Mini = A8Mini::connect().await?;

        cam.send_command_blind(control::A8MiniSimpleCommand::RotateLeft).await?;
        sleep(Duration::from_millis(500));

        cam.send_command_blind(control::A8MiniSimpleCommand::RotateRight).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniSimpleCommand::RotateLeft).await?;
        sleep(Duration::from_millis(500));

        cam.send_command_blind(control::A8MiniSimpleCommand::StopRotation).await?;

        cam.send_command_blind(control::A8MiniSimpleCommand::RotateUp).await?;
        sleep(Duration::from_millis(500));

        cam.send_command_blind(control::A8MiniSimpleCommand::RotateDown).await?;
        sleep(Duration::from_millis(500));

        cam.send_command_blind(control::A8MiniSimpleCommand::StopRotation).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniSimpleCommand::AutoCenter).await?;
        Ok(())
    }

    #[ignore]
    #[tokio::test]
    async fn test_send_complex_commands_blind() -> anyhow::Result<()> {
        let cam: A8Mini = A8Mini::connect().await?;

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchSpeed(50, 50)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchSpeed(50, 10)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchSpeed(-25, -15)).await?;
        sleep(Duration::from_millis(6000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchSpeed(0, 0)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchAngle(90, 0)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchAngle(90, -90)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchAngle(-90, -90)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchAngle(-90, 0)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniComplexCommand::SetYawPitchAngle(0, 0)).await?;
        sleep(Duration::from_millis(1000));

        cam.send_command_blind(control::A8MiniSimpleCommand::AutoCenter).await?;
        Ok(())
    }

    #[ignore]
    #[tokio::test]
    async fn test_send_command_with_ack() -> anyhow::Result<()> {
        let cam: A8Mini = A8Mini::connect().await?;
        println!("{:?}", cam.get_attitude_information().await?);
        Ok(())
    }
}
