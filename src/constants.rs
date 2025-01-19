pub const RECV_BUFF_SIZE: u32 = 64;
pub const SERVER_PORT: u32 = 37260; // Gimbal Camera (Server) Port
pub const SERVER_IP: &str = "192.168.144.25"; // Gimbal Camera (Server) IP Addresses

pub const MIN_CMD_SIZE: u32 = 10;
pub const MAX_CMD_SIZE: u32 = 13;

pub const NUM_COMMANDS: usize = 27; // update this if more commands are added !
// pub const MAX_SIZE_CMD_DESCRIPTION: u32 = 40;

pub const COMMAND_DESCRIPTIONS: [&str; NUM_COMMANDS] = [
  "Auto Centering",
  "Rotate Up",
  "Rotate Down",
  "Rotate Right",
  "Rotate Left",
  "Stop Rotation",
  "Zoom +1",
  "Zoom -1",
  "Absolute Zoom (4.5x)",
  "Acquire the Max Zoom Value",
  "Manual Focus +1",
  "Manual Focus -1",
  "Take Pictures",
  "Record Video",
  "Rotate 100 100",
  "Gimbal Status Information",
  "Auto Focus",
  "Acquire Hardware ID",
  "Acquire Firmware Version",
  "Lock Mode",
  "Follow Mode",
  "FPV Mode",
  "Acquire Attitude Data",
  "Set Video Output as HDMI",
  "Set Video Output as CVBS",
  "Turn Off both CVBS and HDMI Output",
  "Read Range from Laser Rangefinder",
];

pub const COMMANDS: [&[i32]; NUM_COMMANDS] = [
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x08,0x01,0xd1,0x12],       // 0  - Auto Centering
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,0x2D,0x3e,0xd1],  // 1  - Rotate Up
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,-0x2D,0xef,0xdf], // 2  - Rotate Down
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,-0x2D,0x00,0x85,0x64], // 3  - Rotate Right
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x2D,0x00,0x4b,0x54],  // 4  - Rotate Left
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,0x00,0xf1,0x24],  // 5  - Stop rotation
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0x01,0x8d,0x64],       // 6  - Zoom +1
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0xFF,0x5c,0x6a],       // 7  - Zoom -1
  &[0x55,0x66,0x01,0x02,0x00,0x01,0x00,0x0F,0x04,0x05,0x60,0xBB],  // 8  - 4.5x
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x16,0xB2,0xA6],            // 9  - Acquire the Max Zoom Value
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0x01,0xde,0x31],       // 10 - Manual Focus +1
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0xff,0x0f,0x3f],       // 11 - Manual Focus -1
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x00,0x34,0xce],       // 12 - Take Pictures
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x02,0x76,0xee],       // 13 - Record Video
  &[0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x64,0x64,0x3d,0xcf],  // 14 - Rotate 100 100
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0a,0x0f,0x75],            // 15 - Gimbal Status Information
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x04,0x01,0xbc,0x57],       // 16 - Auto Focus
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x02,0x07,0xf4],            // 17 - Acquire Hardware ID
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x01,0x64,0xc4],            // 18 - Acquire Firmware Version
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x03,0x57,0xfe],       // 19 - Lock Mode
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x04,0xb0,0x8e],       // 20 - Follow Mode
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x05,0x91,0x9e],       // 21 - FPV Mode
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0d,0xe8,0x05],            // 22 - Acquire Attitude Data
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x06,0xf2,0xae],       // 23 - Set Video Output as HDMI (Only available on A8 mini, restart to take effect)
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x07,0xd3,0xbe],       // 24 - Set Video Output as CVBS (Only available on A8 mini, restart to take effect)
  &[0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x08,0x3c,0x4f],       // 25 - Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect)
  &[0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x15,0xD1,0x96],            // 26 - Read Range from Laser Rangefinder(Low byte in the front, high byte in the back, available on ZT30)
];

pub const COMMAND_SIZES: [u32; NUM_COMMANDS] = [
  11, // 0  - Auto Centering (command size)
  12, // 1  - Rotate Up (command size)
  12, // 2  - Rotate Down (command size)
  12, // 3  - Rotate Right (command size)
  12, // 4  - Rotate Left (command size)
  12, // 5  - Stop Rotation (command size)
  11, // 6  - Zoom +1 (command size)
  11, // 7  - Zoom -1 (command size)
  12, // 8  - Absolute Zoom (4.5x) (command size)
  10, // 9  - Acquire the Max Zoom Value (command size)
  11, // 10 - Manual Focus +1 (command size)
  11, // 11 - Manual Focus -1 (command size)
  11, // 12 - Take Pictures (command size)
  11, // 13 - Record Video (command size)
  12, // 14 - Rotate 100 100 (command size)
  10, // 15 - Gimbal Status Information (command size)
  11, // 16 - Auto Focus (command size)
  10, // 17 - Acquire Hardware ID (command size)
  10, // 18 - Acquire Firmware Version (command size)
  11, // 19 - Lock Mode (command size)
  11, // 20 - Follow Mode (command size)
  11, // 21 - FPV Mode (command size)
  10, // 22 - Acquire Attitude Data (command size)
  11, // 23 - Set Video Output as HDMI (Only available on A8 mini, restart to take effect) (command size)
  11, // 24 - Set Video Output as CVBS (Only available on A8 mini, restart to take effect) (command size)
  11, // 25 - Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect) (command size)
  10  // 26 - Read Range from Laser Rangefinder(Low byte in the front, high byte in the back, available on ZT30) (command size)
];

pub const CRC16_TAB: [u32; 256] = [
  0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
];
