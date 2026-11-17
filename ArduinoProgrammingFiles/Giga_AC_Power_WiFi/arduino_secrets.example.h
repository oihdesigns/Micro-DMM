// Template -- copy to arduino_secrets.h (same folder) and fill in your network.
// arduino_secrets.h is listed in .gitignore, so the password never gets committed.
#define SECRET_SSID "YourNetworkName"
#define SECRET_PASS "YourNetworkPassword"

// Optional fallbacks, up to SECRET_SSID5. On every join the board scans, then
// tries the networks it can see in this order: SECRET_SSID first, then 2, 3 ...
// A network that is out of range is skipped without waiting; one that refuses
// the password falls through to the next. Leave out the ones you don't need.
//#define SECRET_SSID2 "SecondNetwork"
//#define SECRET_PASS2 "SecondPassword"
//#define SECRET_SSID3 "PhoneHotspot"
//#define SECRET_PASS3 "HotspotPassword"

// More networks can be added without reflashing (fw 2.5): the GUI's Wi-Fi tab,
// the web page's Wi-Fi card or !NETADD save up to five in the board's flash,
// tried after these. With no known network in range the board starts its own
// hotspot GigaPower-XXXX at 192.168.3.1 (password on the OLED).
