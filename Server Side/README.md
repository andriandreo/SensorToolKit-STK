# STK - Server Side

The Server Side for the Sensor ToolKit Framework can be deployed in any machine with Wi-Fi connectivity capable of running Docker.

## Containers Deployment

### Docker and Docker Compose

The easiest way of deploying the required Docker images is *via* Docker engine using Docker Compose. 

#### Linux and macOS
On Linux and macOS, their installation and usage is straightforward:
1. Install `docker` and `docker-compose` following the guidelines for your distro (or *via* [Homebrew](https://brew.sh/) on macOS);
1. start Docker engine enabling any `docker.service` and `docker.socket` if needed,
1. `cd` to the `Server Side` directory;
1. run `docker-compose up -d` (`--detached` option to detach from terminal and run in background).

**A few important caveats:**
1. Step (4) may require root privileges to work: run `sudo docker-compose up -d` instead.
1. **BEWARE** of using `sudo` since the `docker-compose.yaml` declaratives will create new directories inside `Server Side/volumes/`. If not created manually beforehand, those directories and subdirectories will have root permissions, which in turn could prevent the container images from running and interacting properly.
1. As suggested: either create the `volumes` subdirectories manually before deploying the containers, or `chmod` the permissions accordingly afterwards.

To **safely stop** server deployment, just run: `sudo docker-compose down` while being `cd` to `Server Side`.

#### Windows
If you face any limitation using Docker and Docker Compose on Windows, you can use [Podman](https://github.com/containers/podman/blob/main/docs/tutorials/podman-for-windows.md) and [Podman Compose](https://podman-desktop.io/docs/compose) alternatively.

## Accessing and Configuring the Web Applications

By default, web apps can be accesed through the corresponding ports stated in the `docker-compose.yaml`:

- If accessing *via* the local server machine, just use any web browser to access `http://localhost:port` (e.g. `http://localhost:9000` or `http://0.0.0.0:9000` for `portainer`).
- If accessing remotely **while connected to the same network than the server**: `IP_of_local_server_machine:port` (e.g. `http://192.168.1.35:9000` for `portainer` if `IP_of_local_server_machine = 192.168.1.35`).

### Portainer (`port = 9000`)

[Portainer](https://www.portainer.io/) is a container manager to control and troubleshoot Docker images. 

The first time you access `portainer`, you will need to create an user and password.

### influxDB (`port = 8086`)

[influxDB](https://www.influxdata.com/) is a database platform specially engineered for time-series data.

Again, you will need to create user/password in the first access. 

Further setup is needed to finish the Server Side setup: at least [create a bucket](https://docs.influxdata.com/influxdb/cloud/admin/buckets/create-bucket/) (e.g. `Open Circuit Potentiometry`) and [generate an API token](https://docs.influxdata.com/influxdb/v2/admin/tokens/create-token/) with read/write access to this bucket.

### Node-RED (`port = 1880`)

[Node-RED](https://nodered.org/) is a low-code programming platform for connecting event-driven applications.

1. Install the [`@rcomanne/node-red-contrib-influxdb`](https://flows.nodered.org/node/@rcomanne/node-red-contrib-influxdb) package inside the web app;
1. import the `Server Side/Node-RED/STKtest_Node-RED_flows.json` flow application file into the `nodered` web app;
1. configure the `influxDB` node with the previously generated read/write token;
1. deploy the application.

### Grafana (`port = 3000`)

[Grafana](https://grafana.com/) is an open, cloud-based, visualization and alerting platform for multitude of case scenarios.

Once again, you will need to create user/password in the first access. 

1. [Add a new influxDB2 datasource connection](https://docs.influxdata.com/influxdb/v2/tools/grafana/) (more [here](https://grafana.com/docs/grafana/latest/datasources/influxdb/configure-influxdb-data-source/));
1. if desired, [import](https://grafana.com/docs/grafana/latest/dashboards/build-dashboards/import-dashboards/) the `Server Side/Grafana/Dashboards/OCP.json` dashboard as an example of the STK framework setup.