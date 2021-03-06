# Air quality monitoring station 
![lang](https://img.shields.io/github/languages/top/sylwesterf/pms3003.svg?style=flat)
![size](https://img.shields.io/github/repo-size/sylwesterf/pms3003.svg?style=flat)
![lastdt](https://img.shields.io/github/last-commit/sylwesterf/pms3003.svg?style=flat)
![rls](https://img.shields.io/github/release-date-pre/sylwesterf/pms3003.svg?style=flat)
![lic](https://img.shields.io/github/license/sylwesterf/pms3003.svg)

This project consists of three parts:
1. Air quality monitoring station based of Raspberry Pi Zero W and PMS3003 sensor
2. Data transfer and storage (AWS MySQL/DynamoDB/S3) 
3. Data visualization (Plotly/Chart.js/R Shiny/Flask hosted on S3/EC2)

https://sylwester.cf/</br>
http://sylwesterf.s3-website.eu-central-1.amazonaws.com/</br>
https://sylwesterf.github.io/

## Installation
#### 1. Air quality monitoring station

##### Raspberry Pi:

```sh
# download project files
sudo git clone https://github.com/sylwesterf/pms3003.git
cd pms3003

# install dependencies
sudo pip install -r requirements.txt
```

Follow RaspberryPi documentation to enable uart: https://www.raspberrypi.org/documentation/configuration/uart.md

Connect PMS3003 to Raspberry Pi as per sensor datasheet:

| PMS3003       | Rpi           |
|           --- |---            |
| VCC           | +5V           |
| GND           | GND           |
| RxD           | TxD           |
| TxD           | RxD           |


Connect DHT11 (3 PIN) to Raspberry Pi as per sensor datasheet:

| PMS3003       | Rpi           |
|           --- |---            |
| VCC           | +3.3V         |
| OUT           | GPIO7 (BCM)   |
| GND           | GND           |

```sh
# run a test - output to terminal
sudo python test.py
```

#### 2. Data transfer and storage

```sh
# run aws configure and set AWS Access Key ID and AWS Secret Access Key for DynamoDB/S3 upload
sudo aws configure

# run rpi2dynamodb.py script to load data into DynamoDB and generate a (backup) csv file on Raspberry Pi
# set up the aws (cli) credentials
# verify a project path and DynamoDB table name in rpi2dynamodb.py for csv/DynamoDB output 
sudo python rpi2dynamodb.py

# for automatic archival of csv files into S3 use csv2s3.py (set up variables first)
sudo python csv2s3.py
```
For mysql refer to *pms3003/mysql/* folder.

#### 3. Data visualization 

##### Flask (AWS EC2) - NEW 
```sh
# just add below commands to EC2 user data when launching an instance or ssh into it and run it afterwards
curl https://raw.githubusercontent.com/sylwesterf/pms3003/master/viz/py-new/prep.sh -o prep.sh
sudo bash prep.sh
```

```sh
/opt/pms3003/
.
├── pms3003.py
├── latest.py
├── all.py
├── fun.py
├── assets/
│   └── favicon.ico
├── file.py
├── wsgi.py
├── prep.sh
├── output.csv
├── cron.txt
└── requirements.txt
```

##### Flask (AWS EC2) - OLD
```sh
# just add below commands to EC2 user data when launching an instance or ssh into it and run it afterwards
curl https://raw.githubusercontent.com/sylwesterf/pms3003/master/viz/py/prep.sh -o prep.sh
sudo bash prep.sh
```

##### R-Shiny:
Deploy the contents of *pms3003/viz/R/pms3003/* into your shiny server </br>
For shiny server setup on EC2 follow the *pms3003/viz/R/pms3003/ec2_ubuntu_config_R.sh*

##### Chart.js:
Deploy the contents of *pms3003/viz/js-chart/* and update the url for json src file in script.js </br>
Make sure to enable CORS for S3 bucket as per: https://stackoverflow.com/questions/49493279/react-js-how-to-get-rid-of-cross-origin-error-in-codesandbox </br>
See https://sylwesterf.github.io/ and corresponding repo https://github.com/sylwesterf/sylwesterf.github.io

##### plotly.js:
Deploy the contents of *pms3003/viz/js-plotly/* and update the url for json src file in script.js </br>
See http://sylwesterf.s3-website.eu-central-1.amazonaws.com

#### 4. Extras

Refer to *pms3003/scripts/* for helpful scripts: </br>
- dynamodb-update-table.py (add humidity and temperature data to DynamoDB table)
- dynamodb2json.py (export data to json format)
- index-reset-reload.py (move data from non-indexed to indexed table)
- manual-load-from-csv.py	(perfrom a manual load from csv to DynamoDB)
- power-consumption.sh (reduce RasberryPi W power consumption)

## TODO
- ~~temperature and humidity sensor to viz~~
- ~~a javascript viz~~
- ~~add PM2.5=25 limit threshold line~~
- ~~new layout~~
- case for sensors 
- ~~authentication~~
- sending data to kafka
- contenerize viz
- python 3

## Acknowledgments
- https://github.com/Thomas-Tsai/pms3003-g3
- https://medium.com/@rfreeman/serverless-dynamic-real-time-dashboard-with-aws-dynamodb-a1a7f8d3bc01
- https://github.com/szazo/DHT11_Python
- https://github.com/okomarov/dash_on_flask

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
