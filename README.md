# CanSat Trident Code
 Code used for the CanSat Trident at the ESA European CanSat Competition in Italy, 2022

## What is this?
This is a repository with the files used for programming the CanSat Trident.

## How do I use this?
To copy files to the CanSat use the following command:
```bash
scp -r C:\<YOUR PATH HERE>\Trident-CanSat-Code trident@trident:/home/trident/
```

**Boot test:**
Make a systemd process from the boot test code:
```bash
sudo cp /home/trident/boot-test/blink_leds.service /etc/systemd/system/
systemctl daemon-reload
systemctl start blink_leds.service
```

**Main Program:**
Install missing libraries on the CanSat
Run `flight-code/main.py` with Python 3.x
Adjust config variables in the same file near the top.

**Ground Station:**
Run the sketch_3d_view code with processing 3 or higher. The libraries in the `libraries` folder should be copied to your local processing libraries filder as follows:

```bash
Processing
--> Libraries
----> toxiclibs
----> Unfolding
```

The video feed can be found at `http://10.0.0.16:8080` for example by using VLC Network Stream