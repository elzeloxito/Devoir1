# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from custom_types import Direction
from pacman import GameState
from typing import Any, Tuple,List
import util

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self)->Any:
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state:Any)->bool:
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state:Any)->List[Tuple[Any,Direction,int]]:
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions:List[Direction])->int:
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()



def tinyMazeSearch(problem:SearchProblem)->List[Direction]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem:SearchProblem)->List[Direction]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 1 ICI
    '''
    from util import Stack

    initial_state = problem.getStartState()
    fringe = Stack()
    fringe.push([initial_state,[],''])
    visited_states=dict()
    final_state = None
    while not fringe.isEmpty(): 
        state = fringe.pop()
        visited_states[state[0]]=(state[1],state[2])
        if problem.isGoalState(state[0]):
            final_state = state[0]
            break 
        else:
            for successor_state in problem.getSuccessors(state[0]):
                if successor_state[0] not in visited_states:
                    fringe.push([successor_state[0],state[0],successor_state[1]])

    if final_state == None:
        return[]

    directions = []
    path_state = final_state
    while path_state != initial_state:
        directions.insert(0,visited_states[path_state][1])
        path_state = visited_states[path_state][0]
    return directions
    util.raiseNotDefined()

def breadthFirstSearch(problem:SearchProblem)->List[Direction]:
    """Search the shallowest nodes in the search tree first."""

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 2 ICI
    '''
    from util import Queue

    initial_state = problem.getStartState()
    fringe = Queue()
    fringe.push([initial_state,None,''])
    visited_states={initial_state: [[],""]}
    final_state = None
    while not fringe.isEmpty(): 
        state = fringe.pop()
        if problem.isGoalState(state[0]):
            final_state = state[0]
            break 
        else:
            for successor_state in problem.getSuccessors(state[0]):
                if successor_state[0] not in visited_states:
                    fringe.push([successor_state[0],state[0],successor_state[1]])
                    visited_states[successor_state[0]]=(state[0],successor_state[1])

    if final_state == None:
        return []

    directions = []
    path_state = final_state
    while path_state != problem.getStartState():
        directions.insert(0,visited_states[path_state][1])
        path_state = visited_states[path_state][0]
    return directions
    util.raiseNotDefined()


def uniformCostSearch(problem:SearchProblem)->List[Direction]:
    """Search the node of least total cost first."""

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 3 ICI
    '''
    from util import PriorityQueue

    initial_state = problem.getStartState()
    fringe = PriorityQueue()
    fringe.push(initial_state, 0)
    states_infos = {initial_state: [[], "", 0]}
    final_state = None
    while not fringe.isEmpty():
        state = fringe.pop()
        cost = states_infos[state][2]
        if problem.isGoalState(state):
            final_state = state
            break 
        else:
            for successor_state in problem.getSuccessors(state):
                if successor_state[0] not in states_infos:
                    fringe.update(successor_state[0], successor_state[2] + cost) 
                    states_infos[successor_state[0]] = ([state, successor_state[1], successor_state[2] + cost])
                elif  successor_state[2] + cost < states_infos[successor_state[0]][2]:
                    fringe.update(successor_state[0], successor_state[2] + cost) 
                    states_infos[successor_state[0]] = ([state, successor_state[1], successor_state[2] + cost])
    
    if final_state == None:
        return []

    directions = []
    path_state = final_state
    while path_state != problem.getStartState():
        directions.insert(0,states_infos[path_state][1])
        path_state = states_infos[path_state][0]
    return directions

    util.raiseNotDefined()


def nullHeuristic(state:GameState, problem:SearchProblem=None)->List[Direction]:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem:SearchProblem, heuristic=nullHeuristic)->List[Direction]:
    """Search the node that has the lowest combined cost and heuristic first."""
    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 4 ICI
    '''
    from util import PriorityQueue
    
    initial_state = problem.getStartState()
    fringe = PriorityQueue()
    fringe.push(initial_state, 0+heuristic(initial_state,problem))
    infos = {initial_state: [[], "", 0]} #pour le moment pas de cout associe
    final_state = None
    while not fringe.isEmpty():
        state = fringe.pop()
        cost = infos[state][2] #ici le cout reste le même
        if problem.isGoalState(state):
            final_state = state
            break 
        else: # ici la definition du cout pour le successeur contient l'heuristique, ponderation 50/50
            for successor_state in problem.getSuccessors(state):
                if successor_state[0] not in infos:
                    fringe.update(successor_state[0], successor_state[2] + cost + heuristic(successor_state[0],problem)) #on rajoute l'heuristique
                    infos[successor_state[0]] = ([state, successor_state[1], successor_state[2] + cost])
                elif  successor_state[2] + cost  < infos[successor_state[0]][2]:
                    fringe.update(successor_state[0], successor_state[2] + cost+ heuristic(successor_state[0],problem)) 
                    infos[successor_state[0]] = ([state, successor_state[1], successor_state[2] + cost])
    
    if final_state == None:
        return []

    directions = []
    path_state = final_state
    while path_state != problem.getStartState():
        directions.insert(0,infos[path_state][1])
        path_state = infos[path_state][0]
    return directions

    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
