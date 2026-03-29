#read Python version from theDoc file and store in string

version = 'unknown'
#depending on from where we search, we import version ... (used from code generators and from setup.py!)
try:
    with open('../../../docs/theDoc/version.txt', 'r') as file:
        version = file.read().strip()
except FileNotFoundError:
    try:
        with open('../../docs/theDoc/version.txt', 'r') as file:
            version = file.read().strip()
    except FileNotFoundError:
        try:
            with open('../docs/theDoc/version.txt', 'r') as file:
                version = file.read().strip()
        except FileNotFoundError:
            print(f"Error: The file 'version.txt' could not be found; check paths to docs/theDoc/")
except Exception as e:
    print(f"An error occurred: {e}")
        
exudynVersionString = version

