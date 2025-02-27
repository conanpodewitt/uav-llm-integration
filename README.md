USEFULL COMMANDS

```bash
ls /dev/ttyUSB*
```

```bash
sudo chmod 666 /dev/ttyUSB*
```

```bash
sudo evtest
```

```bash
sudo chmod 666 /dev/input/event*
```

```bash
lsusb
```

```bash
sudo chmod 666 /dev/bus/usb/00*/0**
```

Use the following link to enable USB passthrough when running simulations on WSL:
https://github.com/dorssel/usbipd-win/releases/tag/v4.4.0

```powershell
usbipd list
```

PS4 controller might be listed as only usb input device instead of Wireless Controller

```powershell
usbipd bind --busid *-*
```

```powershell
usbipd attach --wsl --busid *-*
```
