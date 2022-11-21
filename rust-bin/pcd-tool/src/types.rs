#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FileFormat {
    LibpclPcd,
    NewslabPcd,
    Pcap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LidarType {
    Vlp16,
    Vlp32,
}
