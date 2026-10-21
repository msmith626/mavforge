# mavforge-sitl

ArduCopter SITL image used by MAVForge's `-m sitl` tests. See [docs/sitl.md](../../docs/sitl.md).

```sh
docker build -t mavforge-sitl:4.7.2 docker/sitl
docker run --rm -p 127.0.0.1:5760:5760 mavforge-sitl:4.7.2
mavforge -c tcp:127.0.0.1:5760 --timeout 30 info
```
