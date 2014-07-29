import httplib
import sys

def main():
    client = httplib.HTTPConnection("localhost:" + str(port))
    client.connect()
    client.request("GET", "/?rate=" + rate)
    client.close()

if __name__ == "__main__":
    rate = sys.argv[1]
    port = sys.argv[2]
    main()