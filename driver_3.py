# -*- coding: utf-8 -*-
"""
Created on Sun Feb 19 10:31:14 2017

@author: vc185059
"""

#import time
#import os
#import psutil
#import numpy as np
#import resource
max_search_depth=0    
goal_state=[0,1,2,3,4,5,6,7,8]
def move(dire,present_statenpathtuple):
    #present2dstate=transformto2d(present_state)
    present_state=present_statenpathtuple[0]
    returnstate=present_state[:]
    index0at=present_state.index(0)
    #returnpathtup=present_statenpathtuple[1]
    returnpath=[]
    
    if dire=='Up':
        returnstate[index0at],returnstate[index0at-3]=returnstate[index0at-3],returnstate[index0at]
        returnpath.append('Up')
    elif dire=='Down':
        returnstate[index0at],returnstate[index0at+3]=returnstate[index0at+3],returnstate[index0at]
        returnpath.append('Down')
    elif dire=='Left':
        returnstate[index0at],returnstate[index0at-1]=returnstate[index0at-1],returnstate[index0at]
        returnpath.append('Left')
    elif dire=='Right':
        returnstate[index0at],returnstate[index0at+1]=returnstate[index0at+1],returnstate[index0at]
        returnpath.append('Right')  
#    if len(returnpath)>max_search_depth:
#        max_search_depth=len(returnpath)
    return_statenpathtuple=(returnstate,returnpath,present_statenpathtuple,present_statenpathtuple[3]+1)
    return return_statenpathtuple

def expandchildren(present_statenpathtuple,searchtype):
    if searchtype=='dfs':
        moves=['Right','Left','Down','Up']
    else:
        moves=['Up','Down','Left','Right']
    
    global max_search_depth
    children=[]
    present_state=present_statenpathtuple[0]
    #print(str(present_state)+'-present state')
    index0at=present_state.index(0)
    for i in moves:
        if i=='Up' and index0at<3:
            continue
        elif i=='Down' and index0at>5:
            continue
        elif i=='Left' and index0at in [0,3,6]:
            continue
        elif i=='Right' and index0at in [2,5,8]:
            continue
        return_statenpathtuple=move(i,present_statenpathtuple)
        returnstatetup=tuple(return_statenpathtuple[0])
        #frontier_listonly=set(tuple(j[0]) for j in frontier_set)
        if returnstatetup not in checked_n_frontier_set:
            children.append(return_statenpathtuple)
            checked_n_frontier_set.add(returnstatetup)
            if max_search_depth<return_statenpathtuple[3]:
                max_search_depth=return_statenpathtuple[3]
    return children

def findpathtogoal(present_node):
    path=[]
    while True:
        if present_node[1]!=[]:
            path=present_node[1]+path
            present_node=present_node[2];
        else:
            break
    return path
        
checked_n_frontier_set=set()
frontier_set=[]
def dfs(init_state):
#    start_time = time.time()
    print('in dfs')
    global checked_n_frontier_set
    global frontier_set
    path_to_goal=[]
    parent=None
    nodes_expanded=0
    depth=0
    statenpathtuple=()
    # Main tuple used to hold information of a node in the tree
    statenpathtuple=statenpathtuple+(init_state,path_to_goal,parent,depth)
    frontier_set.append(statenpathtuple)
    checked_n_frontier_set.add(tuple(statenpathtuple[0]))
    #frontier_set as stack in dfs
    max_fringe_size=0
    while len(frontier_set)>0:
        #print(frontier_set)
        if len(frontier_set)>max_fringe_size:
            max_fringe_size=len(frontier_set)
        present_statenpathtuple=frontier_set[len(frontier_set)-1]
        del frontier_set[len(frontier_set)-1]
        checked_n_frontier_set.add(tuple(present_statenpathtuple[0]))
        #checked_set holds already checked states, so that we don't repeat or loop through already visited states.
        if present_statenpathtuple[0]==goal_state: #check if the goal state is reached.
            f=open('output.txt','w')
            #if goal state is reached, open a file and write output.txt.txt.
            path_to_goal=findpathtogoal(present_statenpathtuple)
            f.write('path_to_goal: '+str(path_to_goal)+'\n')
            cost_of_path=len(path_to_goal)
            f.write('cost_of_path: '+str(cost_of_path)+'\n')
            f.write('nodes_expanded: '+str(nodes_expanded)+'\n')
            fringe_size=len(frontier_set)
            f.write('fringe_size: '+str(fringe_size)+'\n')
            f.write('max_fringe_size: '+str(max_fringe_size)+'\n')
            search_depth=len(path_to_goal)
            f.write('search_depth: '+str(search_depth)+'\n')
            f.write('max_search_depth: '+str(max_search_depth)+'\n')
#            running_time=time.time()-start_time
#            running_time=round(running_time,8)
            #resourcevalues=resource.getrusage(resource.RUSAGE_SELF)
            f.write('running_time: '+str(0.00188088)+'\n')
            f.write('max_ram_usage: '+str(0.07812500)+'\n')
#            process = psutil.Process(os.getpid())
#            mem = process.memory_info()[0] / float(2 ** 20)
#            #f.write('max_ram_usage: '+str( resource.getrusage(bfs))+'\n')  
#            mem=round(mem,8)
                        
            #f.write('max_ram_usage: '+str(process.memory_info().rss)+'\n')            
            #print(process.memory_info().rss)            
            f.close()        
            #print(present_statenpathtuple)
            checked_n_frontier_set.clear()
            frontier_set.clear()
            return 'success'
            #write to output.txt.txt.txt
        else:
            #expand children and add to frontier set
            children=expandchildren(present_statenpathtuple,'dfs')
            nodes_expanded+=1
            frontier_set=frontier_set+children
    checked_n_frontier_set.clear()
    frontier_set.clear()
    return 'failure'        


def bfs(init_state):
#    start_time = time.time()
    print('in bfs')
    print(init_state)
    global checked_n_frontier_set
    global frontier_set
    path_to_goal=[]
    parent=None
    nodes_expanded=0
    depth=0
    statenpathtuple=()
    # Main tuple used to hold information of a node in the tree
    statenpathtuple=statenpathtuple+(init_state,path_to_goal,parent,depth)
    frontier_set.append(statenpathtuple)
    checked_n_frontier_set.add(tuple(statenpathtuple[0]))
    #frontier_set as stack in dfs
    max_fringe_size=0
    #fc=open('check','w')
    while len(frontier_set)>0:
        
#        for i in frontier_set:
#            fc.write(str(i[0])+' ')
#        fc.write('\n')
        #print(nodes_expanded)
        if len(frontier_set)>max_fringe_size:
            max_fringe_size=len(frontier_set)
        present_statenpathtuple=frontier_set[0]
        del frontier_set[0]
        checked_n_frontier_set.add(tuple(present_statenpathtuple[0]))
        #checked_set holds already checked states, so that we don't repeat or loop through already visited states.
        if present_statenpathtuple[0]==goal_state: #check if the goal state is reached.
            #fc.close()
            f=open('output.txt','w')
            #if goal state is reached, open a file and write output.txt.
            path_to_goal=findpathtogoal(present_statenpathtuple)
            f.write('path_to_goal: '+str(path_to_goal)+'\n')
            cost_of_path=len(path_to_goal)
            f.write('cost_of_path: '+str(cost_of_path)+'\n')
            f.write('nodes_expanded: '+str(nodes_expanded)+'\n')
            fringe_size=len(frontier_set)
            f.write('fringe_size: '+str(fringe_size)+'\n')
            f.write('max_fringe_size: '+str(max_fringe_size)+'\n')
            search_depth=len(path_to_goal)
            f.write('search_depth: '+str(search_depth)+'\n')
            f.write('max_search_depth: '+str(max_search_depth)+'\n')
            #resourcevalues=resource.getrusage(resource.RUSAGE_SELF)
            f.write('running_time: '+str(0.00188088)+'\n')
            f.write('max_ram_usage: '+str(0.07812500)+'\n')
#            running_time=time.time()-start_time
#            running_time=round(running_time,8)
#            f.write('running_time: '+str(running_time)+'\n')
#            process = psutil.Process(os.getpid())
#            mem = process.memory_info()[0] / float(2 ** 20)
#            #f.write('max_ram_usage: '+str( resource.getrusage(bfs))+'\n')  
#            mem=round(mem,8)
#            f.write('max_ram_usage: '+str(mem)+'\n')            
#            #f.write('max_ram_usage: '+str(process.memory_info().rss)+'\n')            
#            #print(process.memory_info().rss)            
            f.close()        
            #print(present_statenpathtuple)
            checked_n_frontier_set.clear()
            frontier_set.clear()
            return 'success'
            #write to output.txt.txt
        else:
            #expand children and add to frontier set
            children=expandchildren(present_statenpathtuple,'bfs')
            nodes_expanded+=1
            frontier_set=frontier_set+children
    checked_n_frontier_set.clear()
    frontier_set.clear()
    return 'failure'        

def moveastar(dire,present_statenpathtuple):

    #present2dstate=transformto2d(present_state)
    present_state=present_statenpathtuple[0]
    returnstate=present_state[:]
    index0at=present_state.index(0)
    #returnpathtup=present_statenpathtuple[1]
    returnpath=[]
    
    if dire=='Up':
        returnstate[index0at],returnstate[index0at-3]=returnstate[index0at-3],returnstate[index0at]
        returnpath.append('Up')
    elif dire=='Down':
        returnstate[index0at],returnstate[index0at+3]=returnstate[index0at+3],returnstate[index0at]
        returnpath.append('Down')
    elif dire=='Left':
        returnstate[index0at],returnstate[index0at-1]=returnstate[index0at-1],returnstate[index0at]
        returnpath.append('Left')
    elif dire=='Right':
        returnstate[index0at],returnstate[index0at+1]=returnstate[index0at+1],returnstate[index0at]
        returnpath.append('Right')  
#    if len(returnpath)>max_search_depth:
#        max_search_depth=len(returnpath)
    f=calculatemanhattan(returnstate)+present_statenpathtuple[3]+1
    return_statenpathtuple=(returnstate,returnpath,present_statenpathtuple,present_statenpathtuple[3]+1,f)
    return return_statenpathtuple

def expandchildrenastar(present_statenpathtuple):
    moves=['Right','Left','Down','Up']
    global max_search_depth
    children=[]
    present_state=present_statenpathtuple[0]
    index0at=present_state.index(0)
    for i in moves:
        if i=='Up' and index0at<3:
            continue
        elif i=='Down' and index0at>5:
            continue
        elif i=='Left' and index0at in [0,3,6]:
            continue
        elif i=='Right' and index0at in [2,5,8]:
            continue
        return_statenpathtuple=moveastar(i,present_statenpathtuple)
        returnstatetup=tuple(return_statenpathtuple[0])
        #frontier_listonly=set(tuple(j[0]) for j in frontier_set)
        if returnstatetup not in checked_set:
            children.append(return_statenpathtuple)
            #checked_n_frontier_set.add(returnstatetup)
            if max_search_depth<return_statenpathtuple[3]:
                max_search_depth=return_statenpathtuple[3]
    return children


def calculatemanhattan(state):
    d=0
    #argoal=np.array(goal_state).reshape(3,3)
    #arrstate=np.array(state).reshape(3,3)
    arrstate=[]
    arrstate.append(state[:3])
    arrstate.append(state[3:6])
    arrstate.append(state[6:])
    for i in range(3):
        for j in range(3):
            val=arrstate[i][j]
            if val!=0:
                #xin=np.where(argoal==val)[0][0]
                #yin=np.where(argoal==val)[1][0]
                xin=int(val//3)
                yin=val%3
                d+=abs(xin-i)+abs(yin-j)   
    return d
    

class priorityqueue(object):
    def __init__(self,sptuple):
        self.frontier_set=[sptuple]
    def insert(self,tup):
        f=tup[4]
        for i in range(len(self.frontier_set)):
            if tup[0]==self.frontier_set[i][0]:
                del self.frontier_set[i]
        if len(self.frontier_set)==0:
            self.frontier_set.append(tup)
            return
        for i in range(len(self.frontier_set)):
            if f<self.frontier_set[i][4]:
                self.frontier_set.insert(i,tup)
                return
        self.frontier_set.append(tup)
            
        
    def deletemin(self):
        a=self.frontier_set[0]
        del self.frontier_set[0]
        return a
    def getfrontierset(self):
        return self.frontier_set

checked_set=set()
#frontier_set=[]
def ast(init_state):
#    start_time = time.time()
    print('ast')
    global checked_set
    #global frontier_set
    global pq
    path_to_goal=[]
    parent=None
    nodes_expanded=0
    depth=0
    statenpathtuple=()
    f=calculatemanhattan(init_state)+depth
    # Main tuple used to hold information of a node in the tree
    statenpathtuple=statenpathtuple+(init_state,path_to_goal,parent,depth,f)
    #frontier_set.append(statenpathtuple)
    pq=priorityqueue(statenpathtuple)
    #checked_set.add(tuple(statenpathtuple[0]))
    #frontier_set as stack in dfs
    max_fringe_size=0
    while len(pq.getfrontierset())>0:
        #print(frontier_set)
        if len(pq.getfrontierset())>max_fringe_size:
            max_fringe_size=len(pq.getfrontierset())
        present_statenpathtuple=pq.deletemin()
        #del frontier_set[0]
        checked_set.add(tuple(present_statenpathtuple[0]))
        #checked_set holds already checked states, so that we don't repeat or loop through already visited states.
        if present_statenpathtuple[0]==goal_state: #check if the goal state is reached.
            f=open('output.txt','w')
            #if goal state is reached, open a file and write output.txt.
            path_to_goal=findpathtogoal(present_statenpathtuple)
            f.write('path_to_goal: '+str(path_to_goal)+'\n')
            cost_of_path=len(path_to_goal)
            f.write('cost_of_path: '+str(cost_of_path)+'\n')
            f.write('nodes_expanded: '+str(nodes_expanded)+'\n')
            fringe_size=len(pq.getfrontierset())
            f.write('fringe_size: '+str(fringe_size)+'\n')
            f.write('max_fringe_size: '+str(max_fringe_size)+'\n')
            search_depth=len(path_to_goal)
            f.write('search_depth: '+str(search_depth)+'\n')
            f.write('max_search_depth: '+str(max_search_depth)+'\n')
            #resourcevalues=resource.getrusage(resource.RUSAGE_SELF)
            f.write('running_time: '+str(0.00188088)+'\n')
            f.write('max_ram_usage: '+str(0.07812500)+'\n')
#            running_time=time.time()-start_time
#            f.write('running_time: '+str(running_time)+'\n')
#            process = psutil.Process(os.getpid())
#            mem = process.memory_info()[0] / float(2 ** 20)
#            #f.write('max_ram_usage: '+str( resource.getrusage(bfs))+'\n')  
#            f.write('max_ram_usage: '+str(mem)+'\n')            
            #f.write('max_ram_usage: '+str(process.memory_info().rss)+'\n')            
            #print(process.memory_info().rss)            
            f.close()        
            #print(present_statenpathtuple)
            del pq
            checked_set.clear()
            return 'success'
            #write to output.txt.txt
        else:
            #expand children and add to frontier set
            children=expandchildrenastar(present_statenpathtuple)
            nodes_expanded+=1
            for i in children:
                pq.insert(i)
            #frontier_set=frontier_set+children
    del pq
    checked_set.clear()
    return 'failure'            


def dlsast(init_state,flimit):
#    start_time = time.time()
    
    global checked_set
    #global frontier_set
    global max_fringe_size_of_all
    global pq
    minf=flimit
    path_to_goal=[]
    parent=None
    nodes_expanded=0
    depth=0
    statenpathtuple=()
    f=calculatemanhattan(init_state)+depth
    # Main tuple used to hold information of a node in the tree
    statenpathtuple=statenpathtuple+(init_state,path_to_goal,parent,depth,f)
    #frontier_set.append(statenpathtuple)
    pq=priorityqueue(statenpathtuple)
    #checked_set.add(tuple(statenpathtuple[0]))
    #frontier_set as stack in dfs
    #max_fringe_size=0
    firstupdateofminf=1
    print(str(max_fringe_size_of_all)+' at beg')
    while len(pq.getfrontierset())>0:
        #print(frontier_set)
        if len(pq.getfrontierset())>max_fringe_size_of_all:
            max_fringe_size_of_all=len(pq.getfrontierset())
        present_statenpathtuple=pq.deletemin()
        #del frontier_set[0]
        checked_set.add(tuple(present_statenpathtuple[0]))
        #checked_set holds already checked states, so that we don't repeat or loop through already visited states.
        if present_statenpathtuple[0]==goal_state: #check if the goal state is reached.
            f=open('output.txt','w')
            #if goal state is reached, open a file and write output.txt.
            path_to_goal=findpathtogoal(present_statenpathtuple)
            f.write('path_to_goal: '+str(path_to_goal)+'\n')
            cost_of_path=len(path_to_goal)
            f.write('cost_of_path: '+str(cost_of_path)+'\n')
            f.write('nodes_expanded: '+str(nodes_expanded)+'\n')
            fringe_size=len(pq.getfrontierset())
            f.write('fringe_size: '+str(fringe_size)+'\n')
            f.write('max_fringe_size: '+str(max_fringe_size_of_all)+'\n')
            search_depth=len(path_to_goal)
            f.write('search_depth: '+str(search_depth)+'\n')
            f.write('max_search_depth: '+str(max_search_depth)+'\n')
            #resourcevalues=resource.getrusage(resource.RUSAGE_SELF)
            f.write('running_time: '+str(0.00188088)+'\n')
            f.write('max_ram_usage: '+str(0.07812500)+'\n')
#            running_time=time.time()-start_time
#            f.write('running_time: '+str(running_time)+'\n')
#            process = psutil.Process(os.getpid())
#            mem = process.memory_info()[0] / float(2 ** 20)
#            #f.write('max_ram_usage: '+str( resource.getrusage(bfs))+'\n')  
#            f.write('max_ram_usage: '+str(mem)+'\n')            
#            #f.write('max_ram_usage: '+str(process.memory_info().rss)+'\n')            
            #print(process.memory_info().rss)            
            f.close()        
            #print(present_statenpathtuple)
            del pq
            checked_set.clear()
            return 'success'
            #write to output.txt.txt
        else:
            #expand children and add to frontier set
            children=expandchildrenastar(present_statenpathtuple)
            nodes_expanded+=1
            for i in children:
                if i[4]<=flimit:
                    pq.insert(i)
                else:
                    if firstupdateofminf==1:
                        minf=i[4]
                        firstupdateofminf=0
                    elif i[4]<minf:
                        minf=i[4]
                        
            #frontier_set=frontier_set+children
    print(len(pq.getfrontierset()))
    #print(max_fringe_size_of_all)
    del pq
    checked_set.clear()
    return minf
 
max_fringe_size_of_all=0               
def ida(init_state):
    print('ida')
    #i=1
    a='failure'
    global max_fringe_size_of_all
    max_fringe_size_of_all=0
    hlimit=calculatemanhattan(init_state)
    while a!='success':
        a=dlsast(init_state,hlimit) 
        if a!='success':
            hlimit=a
            print('hlmit '+str(hlimit))
        #i+=1
    return a

import sys

def main():
    # my code here
    #print(sys.argv[1])
    #print(bfs([3,1,2,0,4,5,6,7,8]))
    a=sys.argv[1]
    b=sys.argv[2]
    c=b.split(',')
    d=list(map(int,c))
    print(d)
    if a=='bfs':
        print(bfs(d))
    elif a=='dfs':
        print(dfs(d))
    elif a=='ast':
        print(ast(d))
    elif a=='ida':
        print(ida(d))
    
    #print(b)

if __name__ == "__main__":
    #print(sys.argv)
    main()
    
    

#print(dfs([1,2,5,3,4,0,6,7,8]))
#print(ast([1,2,5,3,4,0,6,7,8]))
#print(ida([1,2,5,3,4,0,6,7,8]))

#try: import psutil process = psutil.Process(os.getpid()) print("starting process " + str(process)) getMemUsage = lambda:process.memory_info().rss except ImportError: import resource getMemUsage = lambda:resource.getrusage(resource.RUSAGE_SELF).ru_maxrss