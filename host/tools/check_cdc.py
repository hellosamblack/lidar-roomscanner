import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.sources import SerialSource
from roomscan.decoder import StreamDecoder

def main():
    print("Listening for CDC frames...")
    src = SerialSource()
    decoder = StreamDecoder()
    frames_decoded = 0
    try:
        for _ in range(50):
            data = src.read()
            if data:
                for frame in decoder.feed(data):
                    frames_decoded += 1
                    print(f"Decoded CDC frame: {frame}")
                if frames_decoded >= 5:
                    print("Successfully received and decoded 5 CDC frames!")
                    return
    except KeyboardInterrupt:
        pass
    print(f"Done. Decoded {frames_decoded} frames.")

if __name__ == "__main__":
    main()
