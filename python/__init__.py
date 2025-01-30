import os
from pprint import pprint

def main():
    # Print OS Details
    pprint(f"OS Name: {os.name}\n")
    pprint(f"OS UName: {os.uname()}\m")
    pprint(f"OS Env: {os.environ}\m")
    pprint(f"CWD: {os.getcwd()}\n")

if __name__ == '__main__':
    main()
