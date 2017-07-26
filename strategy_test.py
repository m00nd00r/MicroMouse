import tester_1,os
import pandas as pd

def test(maze, num):
    scores = []
    fails = 0
    count = 0

    while count <= num:
        scores.append(tester_1.main(maze, output = False))
        count += 1

    fails = scores.count(None)
    scores = filter(None,scores)
    worst = max(scores)
    best = min(scores)
    
    return scores,fails,best,worst

def trials(maze = None, num = 100, strategy1 = None, mapper1 = None):
    if strategy1:
        strategy = strategy1
    else:
        strategy = ['random','avoid dead ends','smart map 1']
    if mapper1:
        mapper =  mapper1
    else:
        mapper = ['mapper1','mapper2']
    
    os.environ['TEST_REPORT'] = 'False'
    os.environ['COVERAGE_RESET_THRESHOLD'] = 'goal'
    
    scores = []
    fails = 0
    
    for st in strategy:
        for m in mapper:
            os.environ['MAP_STRATEGY'] = st
            os.environ['MAPPER'] = m
            
            scores, fails, best, worst = test(maze, num)
            
            print 'Running the {} explorer and the {} mapper through {} trials on {}:'.format(st, m, num, maze)
            print 'robot failed to find goal {} times with average score of {}'.format(fails, sum(scores)/len(scores))
            print 'robot\'s best score is {} and worst score is {}.\n'.format(best, worst)
            
    return