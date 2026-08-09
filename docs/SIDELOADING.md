# Installing BetterSplats on your iPhone (no Mac required)

Every CI build produces an **unsigned IPA** (Actions → `ios-app` run →
`BetterSplats-ipa` artifact, or attached to GitHub Releases for tagged
versions). You sign it yourself during installation — that's what the tools
below do.

Requirements: a LiDAR iPhone (iPhone 12 Pro / 13 Pro / 14 Pro / 15 Pro /
16 Pro or any Pro/Pro Max since, running iOS 17+) and a free Apple ID.

## Path A — Sideloadly (recommended, Windows or Linux+iTunes drivers)

1. Install [Sideloadly](https://sideloadly.io) and iTunes (from Apple, not
   the Microsoft Store version, per Sideloadly's docs).
2. Download the IPA artifact from the latest green `ios-app` run and unzip
   it if GitHub wrapped it in a zip.
3. Connect the iPhone by cable, unlock it, tap **Trust** if asked.
4. In Sideloadly: select the IPA, enter your Apple ID, click **Start**.
   Sideloadly signs the IPA with a free development certificate and installs
   it.
5. On the phone: Settings → General → VPN & Device Management → trust your
   Apple ID's developer profile.
6. iOS 16+: enable **Developer Mode** (Settings → Privacy & Security →
   Developer Mode → on, reboot). The app won't launch without it.

### The 7-day rule (free Apple ID)

Free-account signatures expire after **7 days**; the app then refuses to
launch until you re-sideload. Two important notes:

- **Re-sideloading in place keeps your data.** Captured sessions survive as
  long as you don't delete the app.
- Sessions are also visible in the **Files app** (On My iPhone →
  BetterSplats) — copy or AirDrop finished sessions off the phone early and
  often. Deleting the app deletes any sessions you haven't copied out.

Free accounts are limited to 3 sideloaded apps and 10 app-ID registrations
per week.

## Path B — AltStore

[AltStore](https://altstore.io) works the same way but can **auto-refresh**
the 7-day signature over Wi-Fi whenever the phone and the computer running
AltServer share a network. Install AltServer on a PC, install AltStore to
the phone, then add the BetterSplats IPA inside AltStore.

## Path C — Paid Apple Developer account ($99/year)

Signing with a paid account in Sideloadly extends the expiry to **1 year**
and removes the 3-app limit. Same IPA, no rebuild needed.

## Troubleshooting

- *"Unable to verify app"*: the signature expired — re-sideload.
- *App crashes instantly after install*: check Developer Mode is enabled.
- *Sideloadly "Guru Meditation"*: usually an Apple ID session issue; sign
  out/in or use an app-specific password (Apple ID → Sign-In & Security).
- *Camera screen black*: iOS Settings → BetterSplats → allow Camera.
