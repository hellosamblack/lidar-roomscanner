import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.sources import UdpSource
from roomscan.decoder import StreamDecoder

def main():
    print("Listening for UDP frames on port 5000...")
    udp = UdpSource(timeout=2.0)
    decoder = StreamDecoder()
    frames_decoded = 0
    try:
        for _ in range(50):  # Try for a bit
            data = udp.read()
            if data:
                print(f"Received {len(data)} bytes via UDP from {udp.target_ip}")
                for frame in decoder.feed(data):
                    frames_decoded += 1
                    print(f"Decoded frame: {frame}")
                if frames_decoded >= 5:
                    print("Successfully received and decoded 5 UDP frames!")
                    return
    except KeyboardInterrupt:
        pass
    print(f"Done. Decoded {frames_decoded} frames.")

if __name__ == "__main__":
    main()
