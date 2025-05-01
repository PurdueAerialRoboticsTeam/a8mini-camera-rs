#![allow(non_snake_case)]

pub mod camerastatus;
pub mod checksum;
pub mod constants;
pub mod control;

use bincode::deserialize;
use camerastatus::{Connected, Disconnected};
use std::error::Error;
use tokio::{net::UdpSocket, time::timeout};


#[derive(Debug)]
/// Represents the A8Mini camera API with a dedicate UDP socket for both `Command`s and `HTTPQuery`s.
pub struct A8Mini<CameraStatus> {
    pub command_socket: UdpSocket,
    pub http_socket: UdpSocket,

    pub status: CameraStatus,
}

impl A8Mini<Disconnected> {
    /// Connect to and creates a new `A8Mini` using default ip address `192.168.144.25` and default port 37260 and port 82. 
    /// Remote ports are mapped to port 8080 and port 8088 on local.
    pub async fn connect() -> Result<A8Mini<Connected>, Box<dyn Error>> {
        Ok(Self::connect_to(
            constants::CAMERA_IP,
            constants::CAMERA_COMMAND_PORT,
            constants::CAMERA_HTTP_PORT,
            "8080",
            "8088",
        )
        .await?)
    }

    // Connects to and creates a new `A8Mini` given network args.
    pub async fn connect_to (
        camera_ip: &str,
        camera_command_port: &str,
        camera_http_port: &str,
        local_command_port: &str,
        local_http_port: &str,
    ) -> Result<A8Mini<Connected>, Box<dyn Error>> {
        let camera: A8Mini<Connected> = A8Mini::<Connected> {
            command_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_command_port)).await?,
            http_socket: UdpSocket::bind(format!("0.0.0.0:{}", local_http_port)).await?,
            status: Connected {},
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
}

impl A8Mini<Connected> {
    /// Sends a `control::Command` blind. This should be used for all commands that don't have a ACK.
    pub async fn send_command_blind<T: control::Command>(
        &self,
        command: T,
    ) -> Result<(), Box<dyn Error>> {
        let send_len = self.command_socket.send(command.to_bytes().as_slice()).await?;

        if send_len == 0 {
            return Err("No bytes sent.".into());
        }

        Ok(())
    }

    /// Sends a `control::Command` expecting an ACK. Returns received ACK response bytes.
    pub async fn send_command<T: control::Command>(
        &self,
        command: T,
    ) -> Result<[u8; constants::RECV_BUFF_SIZE], Box<dyn Error>> {
        self.send_command_blind(command).await?;
        let mut recv_buffer = [0; constants::RECV_BUFF_SIZE];

        let recv_len = timeout(
            constants::RECV_TIMEOUT,
            self.command_socket.recv(&mut recv_buffer),
        )
        .await??;
        if recv_len == 0 {
            return Err("No bytes received.".into());
        }

        Ok(recv_buffer)
    }

    /// Verify that camera is connected on both command and http sockets
    pub async fn ping(
        &self,
    ) -> bool {
        self.get_attitude_information().await.is_ok() && self.get_photo_information().await.is_ok()
    }

    /// Retrieves attitude information from the camera. 
    pub async fn get_attitude_information(
        &self,
    ) -> Result<control::A8MiniAtittude, Box<dyn Error>> {
        let attitude_bytes = self
            .send_command(control::A8MiniSimpleCommand::AttitudeInformation)
            .await?;
        let attitude_info: control::A8MiniAtittude = deserialize(&attitude_bytes)?;
        Ok(attitude_info)
    }

    /// Retrieves photo count from the camera. 
    pub async fn get_photo_information(
        &self,
    ) -> Result<i32, Box<dyn Error>> {
        self.send_http_query(control::A8MiniSimpleHTTPQuery::GetMediaCountPhotos)
            .await?
            .data
            .count
            .ok_or("Unable to extract photo count.".into())
    }

    /// Sends a `control::HTTPQuery` and returns the corresponding received `control::HTTPResponse`.
    pub async fn send_http_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> Result<control::HTTPResponse, Box<dyn Error>> {
        let response = reqwest::get(query.to_string()).await?;
        let json = response.json::<control::HTTPResponse>().await?;
        Ok(json)
    }

    /// Retrieves an image or video (WIP) from the camera.
    pub async fn send_http_media_query<T: control::HTTPQuery>(
        &self,
        query: T,
    ) -> Result<Vec<u8>, Box<dyn Error>> {
        let response = reqwest::get(query.to_string()).await?;
        let image_bytes = response.bytes().await?;
        Ok(image_bytes.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::*;

    use std::thread::sleep;
    use std::time::Duration;
    use tokio::fs::File;
    use tokio::io::AsyncWriteExt;


    #[ignore]
    #[tokio::test]
    async fn test_take_and_download_photo() -> Result<(), Box<dyn Error>> {
        let cam: A8Mini<Connected> = A8Mini::connect().await?;

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
            .send_http_media_query(control::A8MiniComplexHTTPQuery::GetPhoto(num_pictures as u8))
            .await?;
        File::create("tmp.jpeg")
            .await?
            .write_all(&picture_bytes)
            .await?;

        Ok(())
    }

    #[ignore]
    #[tokio::test]
    async fn test_send_simple_commands_blind() -> Result<(), Box<dyn Error>> {
        let cam: A8Mini<Connected> = A8Mini::connect().await?;

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
    async fn test_send_complex_commands_blind() -> Result<(), Box<dyn Error>> {
        let cam: A8Mini<Connected> = A8Mini::connect().await?;

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
    async fn test_send_command_with_ack() -> Result<(), Box<dyn Error>> {
        let cam: A8Mini<Connected> = A8Mini::connect().await?;
        println!("{:?}", cam.get_attitude_information().await?);
        Ok(())
    }

    #[ignore]
    #[tokio::test]
    async fn aarya_tests() -> Result<(), Box<dyn Error>> {
        let cam: A8Mini<Connected> = A8Mini::connect().await?;
        // cam.send_command_blind(A8MiniComplexCommand::SetYawPitchAngle(0, 900)).await?;
        // cam.send_command_blind(A8MiniSimpleCommand::RecordVideo).await?;
        // println!("{:?}", cam.send_http_query(A8MiniSimpleHTTPQuery::GetMediaCountVideos).await?);

        // cam.send_command_blind(A8MiniComplexCommand::SetCodecSpecs(0, 2, 1920, 1080, 4000, 0)).await?;

        // cam.send_command_blind(A8MiniSimpleCommand::Resolution4k).await?;
        cam.send_command_blind(A8MiniSimpleCommand::RecordVideo).await?;
        // sleep(Duration::from_millis(10000));
        // cam.send_command_blind(A8MiniSimpleCommand::RecordVideo).await?;
        

        Ok(())
    }
}
