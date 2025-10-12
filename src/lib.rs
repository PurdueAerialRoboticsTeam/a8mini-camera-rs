#![allow(non_snake_case)]

use anyhow::anyhow;
use bincode::deserialize;
use tokio::{net::UdpSocket, time::timeout};

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
        let camera: A8Mini = A8Mini {
            command_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_command_port)).await?,
            http_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_http_port)).await?,
        };

        camera
            .command_socket
            .connect(format!("{}:{}", camera_ip, camera_command_port))
            .await?;
        camera
            .http_socket
            .connect(format!("{}:{}", camera_ip, camera_http_port))
            .await?;
        Ok(camera)
    }

    /// Sends a `control::Command` blind. This should be used for all commands that don't have a ACK.
    pub async fn send_command_blind<T: control::Command>(
        &self,
        command: T,
    ) -> anyhow::Result<()> {
        println!(
            "[COMMAND] Sending command with bytes: {:?}",
            command.to_bytes()
        );
        println!(
            "[COMMAND] Sending command with DATA_LEN: {:?} | CMD_ID: {:?}",
            command.to_bytes()[3],
            command.to_bytes()[7]
        );

        let send_len = self.command_socket.send(command.to_bytes().as_slice()).await?;

        if send_len == 0 {
            println!("[COMMAND] No bytes sent.");
            return Err(anyhow!("No bytes sent.".to_string()));
        }

        println!("[COMMAND] Sent {} bytes successfully.", send_len);

        Ok(())
    }

    /// Sends a `control::Command` expecting an ACK. Returns received ACK response bytes.
    pub async fn send_command<T: control::Command>(
        &self,
        command: T,
    ) -> anyhow::Result<[u8; constants::RECV_BUFF_SIZE]> {
        self.send_command_blind(command).await?;
        let mut recv_buffer = [0; constants::RECV_BUFF_SIZE];

        println!("[COMMAND] Waiting for response.");

        let recv_len = timeout(
            constants::RECV_TIMEOUT,
            self.command_socket.recv(&mut recv_buffer),
        )
        .await??;
        if recv_len == 0 {
            println!("[COMMAND] No bytes received.");
            return Err(anyhow!("No bytes received.".to_string()));
        }

        println!(
            "[COMMAND] Response of size {} received successfully: {:?}",
            recv_len, recv_buffer
        );
        Ok(recv_buffer)
    }

    /// Retrieves attitude information from the camera. 
    /// Can be used as a system connectivity check.
    pub async fn get_attitude_information(
        &self,
    ) -> anyhow::Result<control::A8MiniAtittude> {
        let attitude_bytes = self
            .send_command(control::A8MiniSimpleCommand::AttitudeInformation)
            .await?;
        let attitude_info: control::A8MiniAtittude = deserialize(&attitude_bytes)?;
        Ok(attitude_info)
    }

    /// Sends a `control::HTTPQuery` and returns the corresponding received `control::HTTPResponse`.
    pub async fn send_http_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> anyhow::Result<control::HTTPResponse> {
        let response = reqwest::get(query.to_string()).await?;
        println!("[HTTP] Waiting for response.");

        let json = response.json::<control::HTTPResponse>().await?;
        println!("[HTTP] Received response.");
        Ok(json)
    }

    /// Retrieves an image or video (WIP) from the camera.
    pub async fn send_http_media_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> anyhow::Result<Vec<u8>> {
        let response = reqwest::get(query.to_string()).await?;
        println!("[HTTP] Waiting for response.");

        let image_bytes = response.bytes().await?;
        println!("[HTTP] Received response.");
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
