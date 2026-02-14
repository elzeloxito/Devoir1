# searchAgents.py
# ---------------
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


#######              EQUIPE 84                      ######
####### Axel Oddoux - Matricule : 2486891           ######
####### Theo Piliszczuk–Wild - Matricule : 2486799  ######

"""
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

'''
    INSÉREZ VOTRE SOLUTION À LA QUESTION XX ICI
'''


The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError(fn + ' is not a search function in search.py.')
        func = getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError(heuristic + ' is not a function in searchAgents.py or search.py.')
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError(prob + ' is not a search problem type in SearchAgents.py.')
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)

class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function
    """

    def __init__(self, startingGameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner ' + str(corner))
        self._expanded = 0 # DO NOT CHANGE; Number of search nodes expanded
        # Please add any code here which you would like to use
        # in initializing the problem

        '''
            INSÉREZ VOTRE SOLUTION À LA QUESTION 5 ICI
        '''
    

    def getStartState(self):
        """
        Returns the start state (in your state space, not the full Pacman state
        space)
        """

        '''
            INSÉREZ VOTRE SOLUTION À LA QUESTION 5 ICI
        '''

        # L'état initial n'est plus simplement une position sur la map mais on ajoute aussi l'état des nourritures aux coins 
        # False => pas encore mangée 
        corner_state = [False, False, False, False]
        for i,corner in self.corners:
            if self.startingPosition == corner:
                corner_state[i] = True
        return (self.startingPosition, tuple(corner_state))
        
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
        Returns whether this search state is a goal state of the problem.
        """

        '''
            INSÉREZ VOTRE SOLUTION À LA QUESTION 5 ICI
        '''
        
        # L'état final ne dépend plus de la position de Pacman mais de l'état des nourritures
        for corner in self.corners:
            if state[1] == (True, True, True, True): #quelque soit la position, si les etats des corners sont tous True alors c'est un etat final
                return True
        return False

        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
            For a given state, this should return a list of triples, (successor,
            action, stepCost), where 'successor' is a successor to the current
            state, 'action' is the action required to get there, and 'stepCost'
            is the incremental cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            # Add a successor state to the successor list if the action is legal
            # Here's a code snippet for figuring out whether a new position hits a wall:
            #   x,y = currentPosition
            #   dx, dy = Actions.directionToVector(action)
            #   nextx, nexty = int(x + dx), int(y + dy)
            #   hitsWall = self.walls[nextx][nexty]
           
            '''
                INSÉREZ VOTRE SOLUTION À LA QUESTION 5 ICI
            '''

            # Vérifie en plus que l'état successeur est dans un coin pour update l'état des nourritures
            x,y = state[0]
            corners_state = list(state[1])
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                for i,corner in enumerate(self.corners):
                    if corner == nextState:
                        corners_state[i] = True 
                successors.append(((nextState,tuple(corners_state)), action, 1))

        self._expanded += 1 # DO NOT CHANGE
        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions == None: return 999999
        x,y= self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)

def cornersHeuristic(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 6 ICI
    '''
    # l'heuristique dépend de l'état des coins visités
    '''
       IDEE: retourner la distance manhattan du point le plus près 
       + la distance pour relier les cornes restants dans le meilleure des cas (en ligne droite) 
       On fait une distinction selon le nombre de points restants à visiter,
       ici c'est possible car seulement 4 coins et on connait les emplacements

    # Apparamment s'apparente à une solution MTS lue dans la littérature
    '''
    idx_faux = [i for i, val in enumerate(state[1]) if not val]
    pts_to_visit = sum(1 for val in state[1] if not val) #nombre de points à visiter et leur index
    

    if pts_to_visit == 1: #distance de manhathan si un seul point à visiter
        return ( abs(state[0][0] - corners[idx_faux[0]][0]) + abs(state[0][1] - corners[idx_faux[0]][1])  )
    
    h = [0,0,0,0]
    if state[1][0] == False: 
        h[0] += ( abs(state[0][0] - corners[0][0]) + abs(state[0][1] - corners[0][1])  )

    if state[1][1] == False:
        h[1] += ( abs(state[0][0] - corners[1][0]) + abs(state[0][1] - corners[1][1])  )
    
    if state[1][2] == False:
        h[2] += ( abs(state[0][0] - corners[2][0]) + abs(state[0][1] - corners[2][1])  )
        
    if state[1][3] == False:
        h[3] += ( abs(state[0][0] - corners[3][0]) + abs(state[0][1] - corners[3][1])  )

    if pts_to_visit == 2:
        dist_ptn2ptn = ( abs(corners[idx_faux[0]][0] - corners[idx_faux[1]][0]) + abs(corners[idx_faux[0]][1] - corners[idx_faux[1]][1])  )
        return (min(h[idx_faux[0]],h[idx_faux[1]]) + dist_ptn2ptn) #la distance du point le plus proche + la distance entre les 2 points restants (largeur ou hauteur)
    if pts_to_visit == 3:
        dist_ptn2ptn = corners[3][0] + corners[3][1] -2 # largeur + hauteur
        return (min(h[idx_faux[0]],h[idx_faux[1]],h[idx_faux[2]]) + dist_ptn2ptn)
    if pts_to_visit == 4:
        dist_ptn2ptn = corners[3][0] + corners[3][1] + min(corners[3][0],corners[3][1]) -3 # largeur + hauteur + min(largeur,hauteur)
        return (min(h) + dist_ptn2ptn)
    # La correction de la largeur/longueur avec -3 est pour eviter de compter 2 fois la position
    #  de départ si elle est en (1,1) rend l'heuristique consistante sinon c'est pas le cas!

    return 0 # => in this one 774 nodes expanded

def cornersHeuristic4(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 6 ICI
    '''
    # l'heuristique dépend de l'état des coins visités
    # notre idée c'est de retourner la distance manhattan du point le plus loin seulement 
   
    h = [0,0,0,0]
    
    if state[1][0] == False:
        h[0] += ( abs(state[0][0] - corners[0][0]) + abs(state[0][1] - corners[0][1])  )
        
    if state[1][1] == False:
        h[1] += ( abs(state[0][0] - corners[1][0]) + abs(state[0][1] - corners[1][1])  )
    
    if state[1][2] == False:
        h[2] += ( abs(state[0][0] - corners[2][0]) + abs(state[0][1] - corners[2][1])  )
        
    if state[1][3] == False:
        h[3] += ( abs(state[0][0] - corners[3][0]) + abs(state[0][1] - corners[3][1])  )
        
    return max(h)

def cornersHeuristic3(state, problem):
    
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 6 ICI
    '''
    # l'heuristique dépend de l'état des coins visités
    # notre idée c'est de retourner la distance manhattan totale 
    
    h = 0
    k = 0
    if state[1][0] == False:
        h += ( abs(state[0][0] - corners[0][0]) + abs(state[0][1] - corners[0][1])  )
        k+=1
    if state[1][1] == False:
        h += ( abs(state[0][0] - corners[1][0]) + abs(state[0][1] - corners[1][1])  )
        k+=1
    if state[1][2] == False:
        h += ( abs(state[0][0] - corners[2][0]) + abs(state[0][1] - corners[2][1])  )
        k+=1
    if state[1][3] == False:
        h += ( abs(state[0][0] - corners[3][0]) + abs(state[0][1] - corners[3][1])  )
        k+=1
    
    return h/4 if k!=0 else 0

def cornersHeuristic2(state, problem): #non consistante
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    # l'heuristique dépend de l'état des coins visités
    # notre idée c'est de retourner la distance à vol d'oiseau de chaque point non visité 
    
    h = 0
    if state[1][0] == False:
        h += ( (state[0][0] - corners[0][0])**2 + (state[0][1] - corners[0][1]) **2 )**(0.5)
    if state[1][1] == False:
        h += ( (state[0][0] - corners[1][0])**2 + (state[0][1] - corners[1][1]) **2 )**(0.5)
    if state[1][2] == False:
        h += ( (state[0][0] - corners[2][0])**2 + (state[0][1] - corners[2][1]) **2 )**(0.5)
    if state[1][3] == False:
        h += ( (state[0][0] - corners[3][0])**2 + (state[0][1] - corners[3][1]) **2 )**(0.5)

    return h

def cornersHeuristicGood(state, problem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = problem.corners # These are the corner coordinates
    walls = problem.walls # These are the walls of the maze, as a Grid (game.py)

    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 6 ICI
    '''
    # l'heuristique dépend de l'état des coins visités 
    # notre idée c'est de retourner la distance manhattan du point le plus loin seulement 
   
    h = [0,0,0,0]
    
    if state[1][0] == False:
        h[0] += ( abs(state[0][0] - corners[0][0]) + abs(state[0][1] - corners[0][1])  )
        
    if state[1][1] == False:
        h[1] += ( abs(state[0][0] - corners[1][0]) + abs(state[0][1] - corners[1][1])  )
    
    if state[1][2] == False:
        h[2] += ( abs(state[0][0] - corners[2][0]) + abs(state[0][1] - corners[2][1])  )
        
    if state[1][3] == False:
        h[3] += ( abs(state[0][0] - corners[3][0]) + abs(state[0][1] - corners[3][1])  )
        
    return max(h)   # => in this one 1136 nodes expanded

class AStarCornersAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """
    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0 # DO NOT CHANGE
        self.heuristicInfo = {} # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1 # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem

def foodHeuristic(state, problem: FoodSearchProblem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well. # Je crois que c'est l'inverse mais ce n'est pas grave :)

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    position = list(position)
    
    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 7 ICI
    '''
    
    # 1er test : heuristique qui prend le nombre de nourriture restant sur la map 
    # Resultat : 12507 => 2/5
    '''
    if len(foodGrid.asList())!=1:
        if not problem.isGoalState(state):
            return len(foodGrid.asList())
    else:
        x_pos = position[0]
        y_pos = position[1]

        last_food = foodGrid.asList()
        last_food = last_food[0]

        x_last_food = last_food[0]
        y_last_food = last_food[1]

        return abs(x_pos - x_last_food)+abs(y_pos - y_last_food)
    '''
    # 2e test : heuristique renvoie la moyenne des distances de manhattan entre l'etat et toutes les nourritures restantes
    # Resultat : 11254 => 3/5
    '''
    if len(foodGrid.asList())!=0:
        food_remaining = foodGrid.asList()
        number_items = len(food_remaining)
        mean_distance = 0
        x_pos = position[0]
        y_pos = position[1]
        for food_element in food_remaining:
            x_food = food_element[0]
            y_food = food_element[1]
            mean_distance += abs(x_pos - x_food)+abs(y_pos - y_food)
        mean_distance = 1/number_items*mean_distance
        return mean_distance
    '''
    '''
    if len(foodGrid.asList())!=1: #heuristique Theo
        food_remaining = foodGrid.asList()
        min_distance = 0
        x_pos = position[0]
        y_pos = position[1]
        for food_element in food_remaining:
            x_food = food_element[0]
            y_food = food_element[1]
            temp = abs(x_pos - x_food)+abs(y_pos - y_food)
            if temp > min_distance:
                min_distance = temp
        return min_distance
    '''

    #heuristique Axel  10908 nodes
    # renvoie la distance au point de nourriture le plus proche + le nombre de nourriture restante (-1 pour le premie point déjà mangé)
    '''
    nb_Food = len(foodGrid.asList())
    if nb_Food <=0:
            return 0
    else:
        min_distance = min([util.manhattanDistance(position, food_pos) for food_pos in foodGrid.asList()])
        h1 = min_distance + nb_Food -1
    '''  

    '''
    #heuristique2 Axel => 8204 nodes expanded
    # renvoie la distance au point de nourriture le plus proche 
    # + la distance en x et y entre le point le plus proche et le plus loin
    h2=0
    food_list = foodGrid.asList()
    if not food_list:
        dist2closest, max_dx, max_dy = None, None, None
    else:
        # Distance et indice du point le plus proche
        dist2closest, index_closest = min(
            (util.manhattanDistance(position, food_pos), i)
            for i, food_pos in enumerate(food_list)
        )
        closest_pos = food_list[index_closest]
        # Calcul des distances maximales en x et y par rapport au point le plus proche
        max_dx = max(abs(food_pos[0] - closest_pos[0]) for food_pos in food_list)
        max_dy = max(abs(food_pos[1] - closest_pos[1]) for food_pos in food_list)
        h2 = dist2closest + max_dx + max_dy
    return h2    
    
    
    # Heuristique 3 correspond à la distance au point le plus proche += la distance entre le point le plus proche du point précédent sans les recompter
    food_list = foodGrid.asList()  
    if not food_list:
        return 0
    h3 = 0
    current_position = position
    while food_list:
        # Trouver le point le plus proche non visité
        dist2closest = float('inf')
        index_closest = -1
        for i, food_pos in enumerate(food_list):
            dist = ((current_position[0]-food_pos[0])**2 + (current_position[1]-food_pos[1])**2)**0.5
            if dist < dist2closest:
                dist2closest = dist
                index_closest = i

        closest_pos = food_list[index_closest]

        # Ajouter la distance au total
        h3 += dist2closest
        # Mettre à jour la position actuelle
        current_position = closest_pos
        # Retirer le point visité de la liste
        food_list.pop(index_closest)
    return h3 # on retire 1 pour ne pas compter la distance du point de départ au premier point de nourriture si jamais il y en a une sur la position de départ    
    '''
    # Heuristique 4 : On va faire essayer de comptabiliser les murs => 6313 nodes expanded in 3.7s
    '''
        IDEE: On calcule la distance de manathann du point le plus proche de pacman 
        + on ajoute la distance de manathan de ce point au point le plus loin
        + S'il existe un mur horizontal entre ces 2 points qui bloque toute la ligne
          on ajoute un détour de 4 mouvements, si celui-ci est prolongé on ajoute 
          4 mouvements (en prenant compte des dimensions du plateau, et possibilités)
        + on fait la même chose pour les murs verticaux
        On prend le maximum des détours horizontaux et verticaux (sinon on surestime)

        Défaut : pour une grille grande on fait bcp d'opérations dans le pire des cas, mais pacman ça reste OK
    '''
    food_list = foodGrid.asList()
    hh4 = [0] * foodGrid.height  # Initialisation pour les murs horizontaux
    hv4 = [0] * foodGrid.width  # Initialisation pour les murs verticaux
    
    if not food_list:
        return 0
    else:
        # Distance et indice du point le plus proche
        dist2closest, index_closest = min(
            (util.manhattanDistance(position, food_pos), i)
            for i, food_pos in enumerate(food_list)
        )
        closest_pos = food_list[index_closest]

        # Calcul des distances maximales en x et y du point le plus loin par rapport au point le plus proche
        dist2furthest, index_furthest = max(
             (util.manhattanDistance(closest_pos, food_pos), i)
            for i, food_pos in enumerate(food_list)
        )
        furthest_pos = food_list[index_furthest]
        
        # Vérification des murs horizontaux entre closest_pos et furthest_pos
        for k in range(min(closest_pos[0], furthest_pos[0])+1, max(closest_pos[0], furthest_pos[0])):
            if problem.walls[k][closest_pos[1]]:
                # Vérifie si le mur bloque toute la colonne de y1 à y2
                y_min = min(closest_pos[1], furthest_pos[1])
                y_max = max(closest_pos[1], furthest_pos[1])
                mur_bloque_colonne = True

                
                for y in range(y_min, y_max):
                    if not problem.walls[k][y]:
                        mur_bloque_colonne = False
                        break #pas de blocage

                #Le mur est-il plus grand ?
                if mur_bloque_colonne:
                    hv4[k] += 4  # Détour de 4 mouvements (cf voir comment la grille est construite)
                    Step2Top = min(foodGrid.height-1,y_max) 
                    Step2Bot = max(1,y_min)             #on considère qu'il y a des murs sur les bords
                    
                    for y in range(1,max(Step2Bot,Step2Top)+1):
                        if 0 <= y_min - y and y_max + y <= foodGrid.height-1: 
                            if problem.walls[k][y_min-y] and problem.walls[k][y_max+y]: # top and bot
                                hv4[k] += 4 #détour 
                            else:
                                break #Il faut que le mur soit continu
                        elif 0 <= y_min - y and y_max + y >= foodGrid.height: # top only
                            if problem.walls[k][y_min-y]: 
                                hv4[k] += 4 #détour 
                            else:
                                break
                        elif 0 < y_min - y and y_max + y <= foodGrid.height-1: # bot only
                            if problem.walls[k][y_max + y]: 
                                hv4[k] += 4 #détour 
                            else:
                                break
                        
        # Vérification des murs verticaux entre closest_pos et furthest_pos
        for m in range(min(closest_pos[1], furthest_pos[1])+1, max(closest_pos[1], furthest_pos[1])):
            if problem.walls[closest_pos[0]][m]:

                # Vérifie si le mur bloque toute la ligne de x1 à x2
                x_min = min(closest_pos[0], furthest_pos[0])
                x_max = max(closest_pos[0], furthest_pos[0])
                mur_bloque_row = True

                for x in range(x_min, x_max): 
                    if not problem.walls[x][m]:
                        mur_bloque_row = False
                        break #pas de blocage

                if mur_bloque_row:
                    hh4[m] += 4  # Détour de 2 mouvements
                    
                    Step2Left = max(1,x_min) 
                    Step2Right = min(foodGrid.width-1,x_max)  #on considère qu'il y a des murs sur les bords
                    for x in range(1,max(Step2Right,Step2Left)+1):
                        if 0 <= x_min - x and x_max + x <= foodGrid.width-1: # left and right
                            if problem.walls[x_min-x][m] and problem.walls[x_max+x][m]: 
                                hv4[m] += 4 #détour de 4 mouvements supplémentaires
                            else:
                                break #Il faut que le mur soit continu
                        elif 0 <= x_min - x and x_max + x >= foodGrid.width: # left only
                            if problem.walls[x_min-x][m]: 
                                hv4[m] += 4 
                            else:
                                break
                        elif 0 < x_min - x and x_max + x <= foodGrid.width-1: # right only
                            if problem.walls[x_max+x][m]: 
                                hv4[m] += 4 
                            else:
                                break
                       
    return dist2closest + util.manhattanDistance(closest_pos, furthest_pos) + max(hh4) + max(hv4)
   
    