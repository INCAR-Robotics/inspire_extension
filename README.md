## Starting
1. Source your incar venv. Install by:
```bash
git clone https://github.com/INCAR-Robotics/inspire_extension.git
cd inspire_extension
pip install .
```
2. Add the `inspire_hand_remapping` or any of the other processing steps to the workspace config `command_processing` list.
3. Start and use the incar app as usual. Start the hand with:
```bash
python robot_interface/run.py
```

## Known Issues
If you have issues finding the hand, try the following:

``` bash
systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
systemctl stop brltty.service
systemctl disable brltty.service
```

TODO: Can probably be fixed by udev rules
