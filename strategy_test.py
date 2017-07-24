import tester_1,os

def test(maze):
    count = 0
    scores = []
    fails = 0
    while count <= num:
        scores.append(tester_1.main(maze, output = False))
        count += 1

    fails = scores.count(None)
    scores = filter(None,scores)
    
    return scores,fails

def trials(maze, num = 100):
    scores = []
    fails = 0
    strategy = ['random','avoid dead ends','smart map 1']
    mapper = ['mapper1','mapper2']
    
    os.environ['TEST_REPORT'] = 'False'
    os.environ['COVERAGE_RESET_THRESHOLD'] = 'goal'
    
    for st in strategy:
        for m in mapper:
            os.environ['MAP_STRATEGY'] = st
            os.environ['MAPPER'] = m
            
            scores, fails = test(maze)
            
            print 'Running the {} explorer and the {} mapper through 100 trials on {}:'.format(st, m, maze)
            print 'robot failed to find goal {} times with average of score of {}\n'.format(fails, sum(scores)/len(scores))
            
    return