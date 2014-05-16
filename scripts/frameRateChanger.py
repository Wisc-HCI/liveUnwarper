import httplib
import sys

def main():
	client = httplib.HTTPConnection("localhost:9001")
	client.connect()
	client.request("GET", "/?rate=" + rate)
	client.close()

if __name__ == "__main__":
	rate = sys.argv[1]
	main()