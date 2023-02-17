# Time Synchronization Utils
## Usage
- gpslog.sh
    * `$ sudo su`
    * `$ ./gpslog.sh`

    If it doesn't work, install ts or change ttyUSB0 to the right one. (You can use cat to check the output)

- preprocessing.py
    * `$ python3 preprocessing.py [log file]`

    After running this script, you will get a processed CSV file.
