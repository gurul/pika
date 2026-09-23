# Gates: TF-Luna Mega bench bring-up

- [ ] G1: Receive-only firmware handles valid frames, corruption, resynchronization and unreliable returns and compiles for Mega.
  CHECK: .venv/bin/python scripts/check_mega_lidar.py
  EXPECT: mega lidar verification passed
  EVIDENCE: pending

- [ ] G2: Numbered wiring and logic-level constraints independently reviewed against manufacturer documentation and the user's photo.
  EVIDENCE: pending

- [ ] G3: A bounded physical capture obtains checksum-valid changing target ranges, with no car motion.
  EVIDENCE: pending; user has disconnected lidar wiring.
