//! Minimal RSCN frame parser spike: header + CRC32 + stream demux.
//! Mirrors host/src/roomscan/decoder.py::StreamDecoder byte-for-byte:
//! find magic, unpack header, reject bad version / oversize payload_len
//! (advance 1 byte), check CRC (advance 1 byte on failure), else emit.
use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::time::Instant;

const MAGIC: &[u8; 4] = b"RSCN";
const HEADER_SIZE: usize = 32;
const MAX_PAYLOAD: usize = 1 << 20;
const SUPPORTED_VERSIONS: [u8; 2] = [1, 2];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType { Data = 1, Event = 2, Command = 3, Ack = 4 }

#[derive(Clone, Copy, Debug)]
pub struct FrameHeader {
    pub version: u8,
    pub frame_type: u8,   // raw: unknown types are skipped by consumers, not the framer
    pub stream_id: u8,
    pub flags: u8,
    pub seq: u32,
    pub t_us: u64,
    pub width: u16,
    pub height: u16,
    pub payload_len: u32,
}

#[derive(Debug)]
pub enum HeaderError { BadMagic, UnsupportedVersion(u8) }

impl FrameHeader {
    pub fn unpack(b: &[u8; HEADER_SIZE]) -> Result<Self, HeaderError> {
        if &b[0..4] != MAGIC { return Err(HeaderError::BadMagic); }
        let version = b[4];
        if !SUPPORTED_VERSIONS.contains(&version) { return Err(HeaderError::UnsupportedVersion(version)); }
        Ok(FrameHeader {
            version,
            frame_type: b[5],
            stream_id: b[6],
            flags: b[7],
            seq: u32::from_le_bytes(b[8..12].try_into().unwrap()),
            t_us: u64::from_le_bytes(b[12..20].try_into().unwrap()),
            width: u16::from_le_bytes(b[20..22].try_into().unwrap()),
            height: u16::from_le_bytes(b[22..24].try_into().unwrap()),
            payload_len: u32::from_le_bytes(b[24..28].try_into().unwrap()),
        })
    }
    pub fn frame_type(&self) -> Option<FrameType> {
        match self.frame_type { 1 => Some(FrameType::Data), 2 => Some(FrameType::Event),
                                3 => Some(FrameType::Command), 4 => Some(FrameType::Ack), _ => None }
    }
}

/// Zero-copy frame view: payload borrows the input buffer.
pub struct Frame<'a> { pub header: FrameHeader, pub payload: &'a [u8] }

// ---- CRC-32 (IEEE, reflected, 0xEDB88320), slice-by-8 table -------------------
pub struct Crc32 { t: [[u32; 256]; 8] }
impl Crc32 {
    fn new() -> Self {
        let mut t = [[0u32; 256]; 8];
        for i in 0..256u32 {
            let mut c = i;
            for _ in 0..8 { c = if c & 1 != 0 { 0xEDB88320 ^ (c >> 1) } else { c >> 1 }; }
            t[0][i as usize] = c;
        }
        for i in 0..256 { for k in 1..8 {
            t[k][i] = (t[k - 1][i] >> 8) ^ t[0][(t[k - 1][i] & 0xff) as usize];
        } }
        Crc32 { t }
    }
    fn compute(&self, data: &[u8]) -> u32 {
        let mut c = !0u32;
        let mut chunks = data.chunks_exact(8);
        for ch in &mut chunks {
            let lo = u32::from_le_bytes(ch[0..4].try_into().unwrap()) ^ c;
            let hi = u32::from_le_bytes(ch[4..8].try_into().unwrap());
            c = self.t[7][(lo & 0xff) as usize] ^ self.t[6][((lo >> 8) & 0xff) as usize]
              ^ self.t[5][((lo >> 16) & 0xff) as usize] ^ self.t[4][(lo >> 24) as usize]
              ^ self.t[3][(hi & 0xff) as usize] ^ self.t[2][((hi >> 8) & 0xff) as usize]
              ^ self.t[1][((hi >> 16) & 0xff) as usize] ^ self.t[0][(hi >> 24) as usize];
        }
        for &b in chunks.remainder() { c = self.t[0][((c ^ b as u32) & 0xff) as usize] ^ (c >> 8); }
        !c
    }
}

#[derive(Default, Debug)]
pub struct Stats { pub frames: u64, pub crc_failures: u64, pub bytes_skipped: u64,
                   pub per_stream: BTreeMap<(u8, u8), u64>, pub crc_bytes: u64 }

/// Parse a whole in-memory buffer (a .bin capture is the raw byte stream, no container).
/// Same resync policy as StreamDecoder.feed on a single feed of the whole file.
pub fn parse_all<'a>(buf: &'a [u8], crc: &Crc32, mut on_frame: impl FnMut(&Frame<'a>)) -> Stats {
    let mut st = Stats::default();
    let mut pos = 0usize;
    loop {
        let rel = match memchr_magic(&buf[pos..]) { Some(r) => r, None => {
            let keep = (buf.len() - pos).min(MAGIC.len() - 1);
            st.bytes_skipped += (buf.len() - pos - keep) as u64; return st; } };
        st.bytes_skipped += rel as u64; pos += rel;
        if buf.len() - pos < HEADER_SIZE { return st; }
        let hdr = match FrameHeader::unpack(buf[pos..pos + HEADER_SIZE].try_into().unwrap()) {
            Ok(h) => h, Err(_) => { st.bytes_skipped += 1; pos += 1; continue; } };
        if hdr.payload_len as usize > MAX_PAYLOAD { st.bytes_skipped += 1; pos += 1; continue; }
        let total = HEADER_SIZE + hdr.payload_len as usize + 4;
        if buf.len() - pos < total { return st; }
        let body = &buf[pos..pos + total - 4];
        let wire = u32::from_le_bytes(buf[pos + total - 4..pos + total].try_into().unwrap());
        st.crc_bytes += body.len() as u64;
        if crc.compute(body) != wire { st.crc_failures += 1; st.bytes_skipped += 1; pos += 1; continue; }
        st.frames += 1;
        *st.per_stream.entry((hdr.frame_type, hdr.stream_id)).or_default() += 1;
        on_frame(&Frame { header: hdr, payload: &body[HEADER_SIZE..] });
        pos += total;
    }
}

fn memchr_magic(b: &[u8]) -> Option<usize> { b.windows(4).position(|w| w == MAGIC) }

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 { eprintln!("usage: rscn <file.bin> [reps]"); std::process::exit(2); }
    let reps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let data = fs::read(&args[1]).expect("read");
    let crc = Crc32::new();
    // CRC-only cost: checksum the whole file as one buffer.
    let t0 = Instant::now(); let c0 = std::hint::black_box(crc.compute(std::hint::black_box(&data))); let crc_dt = t0.elapsed(); eprintln!("crc={:08x}", c0);
    let mut st = Stats::default();
    let t1 = Instant::now();
    for _ in 0..reps { st = parse_all(&data, &crc, |_f| {}); }
    let dt = t1.elapsed() / reps as u32;
    println!("file={} bytes={} frames={} crc_failures={} bytes_skipped={}",
             args[1], data.len(), st.frames, st.crc_failures, st.bytes_skipped);
    for ((ft, sid), n) in &st.per_stream { println!("  type={} stream={} count={}", ft, sid, n); }
    let s = dt.as_secs_f64();
    println!("parse: {:.2} ms/file  {:.0} frames/s  {:.4} ms/frame  ({} reps)",
             s * 1e3, st.frames as f64 / s, s * 1e3 / st.frames as f64, reps);
    println!("crc32 whole-file: {:.2} ms -> {:.0} MB/s", crc_dt.as_secs_f64() * 1e3,
             data.len() as f64 / crc_dt.as_secs_f64() / 1e6);
}
