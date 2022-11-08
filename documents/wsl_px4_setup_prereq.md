# WSL Installation Prerequisite

Before following the normal [PX4 setup](ubuntu_px4_setup.md) instructions, edit a WSL file and then restart WSL. This is needed to enable the snap store. 

## 1. Enter the WSL configuration file
```bash
sudo nano /etc/wsl.conf
```

## 2. In the file, type
```
[boot]
systemd=true
```

## 3. Save and exit the file

## 4. Close the WSL terminal and open a Powershell terminal 

## 5. In the Powershell terminal, shut down WSL 
```bash
wsl --shutdown
```

## 6. Open up a WSL terminal and continue along with the [PX4 setup](ubuntu_px4_setup.md)
