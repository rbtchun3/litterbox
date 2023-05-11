#!/usr/bin/env python3


if __name__ == '__main__':
    clist = ''
    with open('./cleanup.txt', 'r') as f:
        while True:

        # Get next line from file
            line = f.readline()
            bag_name = line.strip(" \n")
        # if line is empty
        # end of file is reached
            if not line:
                break
            print('/' + bag_name.replace('-','/')[:bag_name.index(' ')])
    
    print(clist)