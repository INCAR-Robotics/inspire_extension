## Starting
1. Source your incar venv. Install by:
```bash
git clone https://github.com/INCAR-Robotics/inspire_extension.git
cd inspire_extension
pip install .
```
2. In your incar_ws, go to `external/__init__.py` and add a line with the following: `import inspire_extensions`.
3. Add the `inspire_hand_remapping` process step to the workspace config `teleop_processing` list.

4. Start and use the incar app as usual. Start the hand with:
```bash
python incar_start.py
```

## Issues
If you have issues finding the hand, try the following:

``` bash
systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
systemctl stop brltty.service
systemctl disable brltty.service
```

TODO: Can probably be fixed by udev rules