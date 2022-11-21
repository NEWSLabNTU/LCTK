pub(crate) fn escape_bytes(bytes: &[u8]) -> String {
    let escaped: Vec<_> = bytes
        .iter()
        .cloned()
        .flat_map(std::ascii::escape_default)
        .collect();
    String::from_utf8(escaped).unwrap()
}
