
pub trait CameraStatus {}

pub struct Connected {}
pub struct Disconnected {}
pub struct Recording {}

impl CameraStatus for Connected {}
impl CameraStatus for Disconnected {}
impl CameraStatus for Recording {}
