use crate::{checksum, constants};
use serde::{Deserialize, Serialize};
use std::fmt;

/// Trait for camera commands
pub trait Command {
    fn to_bytes(&self) -> Vec<u8>;
}

/// Trait for HTTP API queries
pub trait HTTPQuery {
    fn to_string(&self) -> String;
}

/// Enums for hardcoded simple commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum A8MiniSimpleCommand {
    AutoCenter = 0,   // handled ACK (sta)
    RotateUp = 1,     // handled ACK (sta)
    RotateDown = 2,   // handled ACK (sta)
    RotateRight = 3,  // handled ACK (sta)
    RotateLeft = 4,   // handled ACK (sta)
    StopRotation = 5, // handled ACK (sta)
    ZoomIn = 6,       // handled ACK (sta)
    ZoomOut = 7,      // handled ACK (sta)
    ZoomMax = 8,
    MaxZoomInformation = 9,
    FocusIn = 10,
    FocusOut = 11,
    TakePicture = 12, // no ACK
    RecordVideo = 13, // no ACK
    Rotate100100 = 14,
    CameraInformation = 15,
    AutoFocus = 16, // handled ACK (sta)
    HardwareIDInformation = 17,
    FirmwareVersionInformation = 18,
    SetLockMode = 19,
    SetFollowMode = 20,
    SetFPVMode = 21,
    AttitudeInformation = 22,
    SetVideoOutputHDMI = 23,
    SetVideoOutputCVBS = 24,
    SetVideoOutputOff = 25,
    LaserRangefinderInformation = 26,
    RebootCamera = 27,
    RebootGimbal = 28,
    Resolution4k = 29,
    Heartbeat = 30,
    GimbalStatus = 31,
}

impl Command for A8MiniSimpleCommand {
    fn to_bytes(&self) -> Vec<u8> {
        constants::HARDCODED_COMMANDS[*self as usize].to_vec()
    }
}

/// Enums for commands that require continuous values for data field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum A8MiniComplexCommand {
    SetYawPitchSpeed(i8, i8),
    SetYawPitchAngle(i16, i16),
    SetTimeUTC(u64),
    GetCodecSpecs(u8),                        // TODO: WIP
    SetCodecSpecs(u8, u8, u16, u16, u16, u8), // TODO: WIP
    RequestGimbalDataStream(u8, u8),          // gimbal data stream
}

impl Command for A8MiniComplexCommand {
    fn to_bytes(&self) -> Vec<u8> {
        match *self {
            A8MiniComplexCommand::SetYawPitchSpeed(v_yaw, v_pitch) => {
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07];

                byte_arr.push(v_yaw.clamp(-100, 100) as u8);
                byte_arr.push(v_pitch.clamp(-100, 100) as u8);

                byte_arr.extend_from_slice(&checksum::crc16_calc(&byte_arr, 0));

                byte_arr
            }
            A8MiniComplexCommand::SetYawPitchAngle(theta_yaw, theta_pitch) => {
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0e];

                byte_arr.extend_from_slice(&theta_yaw.clamp(-1350, 1350).to_be_bytes());
                byte_arr.extend_from_slice(&theta_pitch.clamp(-900, 250).to_be_bytes());

                byte_arr.extend_from_slice(&checksum::crc16_calc(&byte_arr, 0));

                byte_arr
            }
            A8MiniComplexCommand::SetTimeUTC(timestamp) => {
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x30];

                byte_arr.extend_from_slice(&timestamp.to_be_bytes());

                byte_arr
            }
            A8MiniComplexCommand::GetCodecSpecs(stream_type) => {
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x20];

                byte_arr.extend_from_slice(&stream_type.clamp(0, 2).to_be_bytes());

                byte_arr
            }
            A8MiniComplexCommand::SetCodecSpecs(
                stream_type,
                video_enc_type,
                resolution_l,
                resolution_h,
                video_bitrate,
                _,
            ) => {
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x21];

                byte_arr.extend_from_slice(&stream_type.clamp(0, 2).to_be_bytes());
                byte_arr.extend_from_slice(&video_enc_type.clamp(1, 2).to_be_bytes());

                // TODO: make sure resolution_l and resolution_h are clamped to only 1920/1280 and 1080/720 respectively
                byte_arr.extend_from_slice(&resolution_l.to_be_bytes());
                byte_arr.extend_from_slice(&resolution_h.to_be_bytes());

                // TODO: make sure video bitrate is reasonable
                byte_arr.extend_from_slice(&video_bitrate.to_be_bytes());

                byte_arr
            }
            // implementation for 0x25 Request Gimbal Data Stream
            A8MiniComplexCommand::RequestGimbalDataStream(data_type, data_freq) => {
                // Header (STX + CTRL + DataLen=2 + Seq=0 + CmdID=0x25)
                let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x25];
                
                byte_arr.push(data_type);
                byte_arr.push(data_freq);
                
                byte_arr.extend_from_slice(&checksum::crc16_calc(&byte_arr, 0));
                byte_arr
            }
        }
    }
}

/// Enums for simple HTTP queries
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum A8MiniSimpleHTTPQuery {
    GetDirectoriesPhotos,
    GetDirectoriesVideos,
    GetMediaCountPhotos,
    GetMediaCountVideos,
}

impl HTTPQuery for A8MiniSimpleHTTPQuery {
    fn to_string(&self) -> String {
        match *self {
            A8MiniSimpleHTTPQuery::GetDirectoriesPhotos => "http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0".to_string(),
            A8MiniSimpleHTTPQuery::GetDirectoriesVideos => "http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=1".to_string(),
            A8MiniSimpleHTTPQuery::GetMediaCountPhotos => "http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getmediacount?media_type=0&path=101SIYI_IMG".to_string(),
            A8MiniSimpleHTTPQuery::GetMediaCountVideos => "http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getmediacount?media_type=1&path=100SIYI_VID".to_string(),
        }
    }
}

/// Enums for complex HTTP queries
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum A8MiniComplexHTTPQuery {
    GetPhoto(u32),
    GetVideo(u32),
}

impl HTTPQuery for A8MiniComplexHTTPQuery {
    fn to_string(&self) -> String {
        match *self {
            A8MiniComplexHTTPQuery::GetPhoto(photo_ind) => format!(
                "http://192.168.144.25:82/photo/101SIYI_IMG/IMG_{:0>4}.jpg",
                photo_ind
            ),
            A8MiniComplexHTTPQuery::GetVideo(video_ind) => format!(
                "http://192.168.144.25:82/photo/100SIYI_VID/REC_{:0>4}.mp4",
                video_ind
            ),
        }
    }
}

/// Response json format
#[derive(Debug, Serialize, Deserialize)]
pub struct HTTPResponse {
    pub code: i32,
    pub data: HTTPResponseData,
    pub success: bool,
    pub message: String,
}

/// Response json data format
#[derive(Debug, Serialize, Deserialize)]
pub struct HTTPResponseData {
    pub media_type: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub directories: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub count: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub list: Option<String>,
}

#[derive(Debug, PartialEq, Eq, Deserialize)]
pub struct A8MiniFirmwareVersion {
    // Camera Code Version (Bytes 8-11)
    pub code_ver_byte0: u8, // Patch
    pub code_ver_byte1: u8, // Minor
    pub code_ver_byte2: u8, // Major
    pub code_ver_byte3: u8, // Rev

    // Gimbal Version (Bytes 12-15)
    pub gimbal_ver_byte0: u8, // Patch (e.g. 4)
    pub gimbal_ver_byte1: u8, // Minor (e.g. 4)
    pub gimbal_ver_byte2: u8, // Major (e.g. 0)
    pub gimbal_ver_byte3: u8, // Rev   (e.g. 115)
}

impl fmt::Display for A8MiniFirmwareVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Display format: Major.Minor.Patch
        write!(
            f,
            "FIRMWARE VERSION:\n\tCamera: {}.{}.{}\n\tGimbal: {}.{}.{} (Build {})",
            // Camera
            self.code_ver_byte2, self.code_ver_byte1, self.code_ver_byte0,
            // Gimbal: Byte2 is Major (0), Byte1 is Minor (4), Byte0 is Patch (4)
            self.gimbal_ver_byte2, self.gimbal_ver_byte1, self.gimbal_ver_byte0, self.gimbal_ver_byte3
        )
    }
}
/// Camera attitude information
#[derive(Debug, PartialEq, Eq, Deserialize)]
pub struct A8MiniAttitude {
    pub theta_yaw: i16,
    pub theta_pitch: i16,
    pub theta_roll: i16,
    pub v_yaw: i16,
    pub v_pitch: i16,
    pub v_roll: i16,
}

impl fmt::Display for A8MiniAttitude {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let yaw_deg = self.theta_yaw as f32 / 10.0;
        let pitch_deg = self.theta_pitch as f32 / 10.0;
        let roll_deg = self.theta_roll as f32 / 10.0;

        write!(
            f,
            "GIMBAL ATTITUDE:\n\tYaw:   {:.1}°\n\tPitch: {:.1}°\n\tRoll:  {:.1}°\n\t(Speeds: Y={}, P={}, R={})",
            yaw_deg, pitch_deg, roll_deg, self.v_yaw, self.v_pitch, self.v_roll
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complex_command_creation_angle() {
        let computed_command = A8MiniComplexCommand::SetYawPitchAngle(130, -20).to_bytes();
        let expected_command: [u8; 14] = [
            0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x82, 0xff, 0xec, 0x8f, 0xad,
        ];
        assert_eq!(computed_command, expected_command);
    }

    #[test]
    fn test_complex_command_creation_speed() {
        let computed_command = A8MiniComplexCommand::SetYawPitchSpeed(104, -20).to_bytes();
        let expected_command: [u8; 12] = [
            0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x07, 0x64, 0xec, 0xbd, 0xdf,
        ];
        assert_eq!(computed_command, expected_command);
    }

    #[test]
    fn test_byte_deserialization() {
        let attitude_bytes: &[u8] = &[
            0x28, 0x00, 0x32, 0x00, 0x3c, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00,
        ];

        //little endian deserialize
        let computed_attitude_info: A8MiniAttitude = bincode::deserialize(attitude_bytes).unwrap();

        let expected_attitude_info = A8MiniAttitude {
            theta_yaw: 40,
            theta_pitch: 50,
            theta_roll: 60,
            v_yaw: 4,
            v_pitch: 5,
            v_roll: 6,
        };

        assert_eq!(computed_attitude_info, expected_attitude_info);
    }
}