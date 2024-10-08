Sample config options available to configure :

1. Single channel with one me and ipmb :

```json
{
  "channels": [
    {
      "type": "me",
      "slave-path": "/dev/ipmb-4",
      "bmc-addr": 32,
      "remote-addr": 44
    },
    {
      "type": "ipmb",
      "slave-path": "/dev/ipmb-9",
      "bmc-addr": 32,
      "remote-addr": 96
    }
  ]
}
```

2. Multiple sub channels with me and ipmb :

```json
{
  "channels": [
    {
      "type": "me",
      "slave-path": "/dev/ipmb-1",
      "bmc-addr": 32,
      "remote-addr": 64,
      "devIndex": 0
    },
    {
      "type": "ipmb",
      "slave-path": "/dev/ipmb-3",
      "bmc-addr": 32,
      "remote-addr": 64,
      "devIndex": 0
    },
    {
      "type": "me",
      "slave-path": "/dev/ipmb-5",
      "bmc-addr": 32,
      "remote-addr": 64,
      "devIndex": 1
    },
    {
      "type": "ipmb",
      "slave-path": "/dev/ipmb-7",
      "bmc-addr": 32,
      "remote-addr": 64,
      "devIndex": 1
    }
  ]
}
```

```text
Config fields :

type          : This points to the ChannelType. It can be ME or ipmb channel.
slave-path    : The ipmb device path.
bmc-addr      : This is BMC target address to communicate between BMC and device.
remote-addr   : This is Remote/requester target address to communicate between BMC and device.
devIndex      : This devIndex used to identify the particular device/host.
```
