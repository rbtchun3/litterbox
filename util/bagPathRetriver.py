#!/usr/bin/env python3

'''
example of txt
    hdd5-burro_bags_3-6017-6017-20220121_144529_archive-20220121_144529_archive.bag D: 807.365787912082
    hdd5-burro_bags_3-7002-20210402_121947_archive-20210402_121947_archive.bag D: 203.0684180341141
    hdd5-burro_bags_2023-20230117_202738_archive-20230117_202738_archive.bag D: 211.0921935143681
    hdd5-burro_bags_3-8881-20221114_151524_archive-20221114_151524_archive.bag D: 78.12115232362748
    hdd4-burro_bags_1-6027-20210118_151202_archive-20210118_151202_archive.bag D: 970.9298261190913
    hdd6-burro_bags_2023-20230104_160943_archive-20230104_160943_archive.bag D: 297.94026705690186
    hdd1-burro_bags_2-7002-20210402_121449_archive-20210402_121449_archive.bag D: 201.79734897662658
    hdd5-burro_bags_2023-8883-20230328_115927_archive-20230328_115927_archive.bag D: 104.93462190518262
    hdd5-burro_bags_3-8883-20220922_140231_archive-20220922_140231_archive.bag D: 669.4496699892763
    hdd5-burro_bags_3-6017-20220414_152946_archive-20220414_152946_archive.bag D: 203.48947497420951
    hdd5-burro_bags_2023-20230117_204820_archive-20230117_204820_archive.bag D: 188.57428589175532
    hdd5-burro_bags_3-8881-20221114_160803_archive-20221114_160803_archive.bag D: 87.9439581489756
    hdd1-burro_bags_2-6030-20210319_143420_archive-20210319_143420_archive.bag D: 250.36651397021402
    hdd4-burro_bags_1-6030-6024-20210319_142307_archive-20210319_142307_archive.bag D: 187.84298698912892
'''

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