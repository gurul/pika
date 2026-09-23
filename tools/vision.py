"""Scene understanding from the car's camera through an OpenAI vision model.

Sonar gives geometry. This gives what sonar cannot: the name of the room the
car is in, hazards ahead that a range sensor misses (stairs down, cables,
pets, people, glass), and open doorways. The explorer uses hazards as an
extra veto before a forward move and writes room labels onto the map.

Credentials: OPENAI_API_KEY from ~/.config/car/env, else from
~/.config/cc-buddy-bridge/env. Model: CAR_VISION_MODEL from the same file,
default gpt-5-mini. The key is never logged.
"""
import base64, json, os, threading, time

ENV_FILES = [os.path.expanduser("~/.config/car/env"), os.path.expanduser("~/.config/cc-buddy-bridge/env")]
DEFAULT_MODEL = "gpt-5-mini"
ROOMS = ["kitchen", "living room", "bedroom", "bathroom", "hallway", "office", "dining room", "closet", "garage", "unknown"]
HAZARDS = ["stairs_down", "stairs_up", "cable", "pet", "person", "glass", "liquid", "clutter", "other"]

PROMPT = f"""You are the eyes of a small floor robot (10 cm tall) mapping a house with sonar.
This photo is from its forward camera, tilted slightly upward. Answer with JSON only:
{{"room": one of {ROOMS},
 "hazards": [{{"type": one of {HAZARDS}, "where": "ahead|left|right", "distance_m": number}}],
 "doorways": [{{"where": "ahead|left|right", "open": true|false}}],
 "notes": short sentence}}
Report a hazard only if it is plausibly within 2 metres of the robot's path. Stairs going down are the most important thing to report. If unsure of the room, say "unknown"."""


def load_env():
    env = {}
    for path in ENV_FILES:
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        k, _, v = line.partition("=")
                        env.setdefault(k.strip(), v.strip().strip('"').strip("'"))
        except OSError:
            continue
    return env


class Vision:
    def __init__(self, log=None):
        self.log = log or (lambda *a, **k: None)
        env = load_env()
        self.key = os.environ.get("OPENAI_API_KEY") or env.get("OPENAI_API_KEY")
        self.model = os.environ.get("CAR_VISION_MODEL") or env.get("CAR_VISION_MODEL") or DEFAULT_MODEL
        self.client = None
        self.enabled = False
        self.calls = 0; self.failures = 0
        self.latest = None
        self._busy = threading.Lock()
        if self.key:
            try:
                from openai import OpenAI
                self.client = OpenAI(api_key=self.key)
                self.enabled = True
            except Exception as e:                     # package missing or broken
                self.log("vision", status="disabled", why=f"openai import failed: {e}")
        else:
            self.log("vision", status="disabled", why="no OPENAI_API_KEY in ~/.config/car/env")

    def describe(self, jpeg):
        """Blocking call. Returns a dict or None."""
        if not self.enabled:
            return None
        self.calls += 1
        data_url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()
        try:
            resp = self.client.responses.create(
                model=self.model,
                input=[{"role": "user", "content": [
                    {"type": "input_text", "text": PROMPT},
                    {"type": "input_image", "image_url": data_url},
                ]}],
            )
            text = resp.output_text.strip()
            if text.startswith("```"):
                text = text.strip("`").split("\n", 1)[-1].rsplit("```", 1)[0]
            out = json.loads(text)
            out["room"] = out.get("room", "unknown") if out.get("room") in ROOMS else "unknown"
            out["hazards"] = [h for h in out.get("hazards", []) if isinstance(h, dict) and h.get("type") in HAZARDS]
            out["doorways"] = [d for d in out.get("doorways", []) if isinstance(d, dict)]
            return out
        except Exception as e:
            self.failures += 1
            self.log("vision", status="error", why=str(e)[:160])
            if self.failures >= 3 and self.calls == self.failures:
                self.enabled = False
                self.log("vision", status="disabled", why="three failures in a row at startup")
            return None

    def describe_async(self, jpeg, tag):
        """Run describe() in the background; result lands in self.latest as (tag, dict)."""
        if not self.enabled or not self._busy.acquire(blocking=False):
            return False
        def run():
            try:
                out = self.describe(jpeg)
                if out is not None:
                    self.latest = (tag, out, time.monotonic())
                    self.log("vision", tag=tag, room=out["room"], hazards=[h["type"] + "@" + h.get("where", "?") for h in out["hazards"]],
                             doorways=[d.get("where", "?") + (":open" if d.get("open") else ":closed") for d in out["doorways"]], notes=out.get("notes", "")[:120])
            finally:
                self._busy.release()
        threading.Thread(target=run, daemon=True).start()
        return True

    VETO_TYPES = {"stairs_down", "person", "pet", "liquid"}   # sonar and the floor sensors cannot see these

    @staticmethod
    def hazard_ahead(out, max_m=1.5):
        """Hazard types close in front that should veto the next forward hop.
        Clutter and cables are left to the sonar: the model reports them in
        almost every indoor frame and vetoing on them stalls exploration."""
        if not out:
            return []
        return [h["type"] for h in out.get("hazards", [])
                if h["type"] in Vision.VETO_TYPES and h.get("where") == "ahead"
                and float(h.get("distance_m", 9) or 9) <= max_m]
