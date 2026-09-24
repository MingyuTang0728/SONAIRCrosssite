# When the console disconnects, or a button does nothing

Two faults used to produce almost every report of this kind, and both are
fixed. This page says what they were, because knowing what a symptom used to
mean is how you tell a new fault from an old one.

## "I pressed a button and it disconnected"

The receive loop ran every handler bare. Any exception — a robot socket
closing mid-command, a reply holding a value `json` could not encode, a field
a newer console sent that an older agent did not expect — escaped the loop
and closed the websocket. The browser saw a dead link and named no cause.

Now each message is wrapped. A fault is logged with its traceback, kept in a
ring buffer, and sent to the page as **the action that failed**. The link
stays up.

**So: look at Connect → Diagnostics.** It names the message that failed and
what the agent said about it. **Copy for a bug report** puts the page state,
the close code and the recent faults on the clipboard.

## "Half the buttons do nothing"

The robot has five separate channels and only one of them used to follow the
address you typed:

| Channel | Port | What it carries |
|---|---|---|
| Telemetry | 30004 (RTDE) | joint angles, forces, temperatures |
| Realtime | 30003 | the fallback position reader |
| URScript | 30002 | every motion command |
| Dashboard | 29999 | power, brake release, load/play/stop, popups |
| Program list | FTP | the `.urp` files on the pendant |

Only telemetry moved. The rest stayed on whatever `UR_IP` held when the agent
started — `192.168.0.20` unless you set the environment variable. So unless
your robot happened to sit at that address, joint angles streamed perfectly
while power on, brake release, load program, play, stop, freedrive and every
I/O control quietly timed out.

That is why it looked random: the page that showed numbers worked, and the
pages that sent commands did not.

**One address now moves all five.** Connect → Robot link → Connect to the
robot. The reply lists the channels it was applied to, and Diagnostics shows
the address in use.

## "It connects and then drops after a while"

The agent used to JPEG-encode colour, depth and *both* infrared streams and
push them every 50 ms to every browser, whatever page it was on — 5.2 MB/s at
the resolutions the calibration needs, encoded on the same thread that
answers the keepalive.

Each page now declares what it displays and gets that alone. Measured:

| Page | Before | After |
|---|---|---|
| Robot, Sensors, Record, Automate | 5.25 MB/s | **0.00 MB/s** |
| Calibrate | 5.25 MB/s | **0.49 MB/s** |
| Camera (all four streams) | 5.25 MB/s | 3.28 MB/s |

The preview is downscaled; the algorithms still read the full frame. If the
browser cannot keep up the agent slows the preview rather than queueing, and
says so in the log.

Diagnostics shows what is being sent, at what rate.

## Checks worth doing in order

1. **Connect → Diagnostics.** Is the robot address the one on the pendant?
   Are there faults listed?
2. **Is the pendant in Remote Control?** It cannot be read back from the
   controller, so nothing can tell you except the symptom: the robot accepts
   the connection and ignores every command.
3. **Automate → Check now.** The pre-flight names, in sentences, everything
   the cell is missing.
