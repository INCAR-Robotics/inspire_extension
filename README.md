If you have issues finding the hand, try the following:
``` bash
systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
systemctl stop brltty.service
systemctl disable brltty.service
```

TODO: Can probably be fixed by udev rules