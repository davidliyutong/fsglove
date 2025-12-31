# Automatically Deployment of RFMocap on Raspberry Pi

## Usage

1. Create an inventory file, e.g. `rpi.ini`

    ```ini
    # rpi.ini
    [rpi]
    192.168.1.2 ansible_user=pi
    ```

2. Apply the playbooks

    ```shell
    ansible-playbook -i rpi.ini manifests/playbook/01_prepare.yml
    ansible-playbook -i rpi.ini manifests/playbook/02_go.yml
    ansible-playbook -i rpi.ini manifests/playbook/03_driver.yml
    ansible-playbook -i rpi.ini manifests/playbook/04_apiserver.yml

    ```

> User interaction is needed for selecting Go mirror (`02_go.yml`) and installation source of usb-uart driver (`03_driver.yml`).
> The playbook will try to install usb-uart driver from Github, with local submodule `third_party/ch9344ser_linux` as fallback option. Make sure the submodule is initialized if you choose to use the local way.
