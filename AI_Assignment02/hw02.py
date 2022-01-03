from util import manhattanDistance
from game import Directions
import random, util
from heapq import *
from game import Agent

## Example Agent
class ReflexAgent(Agent):

  def Action(self, gameState):

    move_candidate = gameState.getLegalActions()

    scores = [self.reflex_agent_evaluationFunc(gameState, action) for action in move_candidate]
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def reflex_agent_evaluationFunc(self, currentGameState, action):

    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    oldFood = currentGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    return successorGameState.getScore()



def scoreEvalFunc(currentGameState):

  return currentGameState.getScore()

class AdversialSearchAgent(Agent):

  def __init__(self, getFunc ='scoreEvalFunc', depth ='2'):
    self.index = 0
    self.evaluationFunction = util.lookup(getFunc, globals())

    self.depth = int(depth)

######################################################################################

class MinimaxAgent(AdversialSearchAgent):
  """
    [문제 01] MiniMax의 Action을 구현하시오. (20점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    total_agent = gameState.getNumAgents()
    evaluation = self.evaluationFunction
    depth_limit = self.depth
    def terminal_state(S,D):
      if S.isWin() or S.isLose():
        return True
      else:
        return False
    def pacman_turn(depth,current,agent):
      if not terminal_state(current, depth):
        moving = current.getLegalActions(agent)
        candidate = []
        for move in moving:
          following = current.generateSuccessor(agent,move)
          heappush(candidate, [(-1) * ghost_turn(depth, following, agent + 1), move])
        if depth == 1:
          return heappop(candidate)[1]
        return heappop(candidate)[0]*(-1)
      else:
        return evaluation(current)

    def ghost_turn(depth,current,agent):
      if not terminal_state(current, depth):
        moving = current.getLegalActions(agent)
        candidate = []
        for move in moving:
          following = current.generateSuccessor(agent, move)
          if agent + 1 == total_agent:
            if depth < depth_limit:
              heappush(candidate,[pacman_turn(depth+1,following,0),move])
            else:
              heappush(candidate, [evaluation(following), move])
          else:
            heappush(candidate,[ghost_turn(depth, following, agent + 1),move])
        return heappop(candidate)[0]

      else:
        return evaluation(current)

    return pacman_turn(1,gameState,0)
    #raise Exception("Not implemented yet")

    ############################################################################




class AlphaBetaAgent(AdversialSearchAgent):
  """
    [문제 02] AlphaBeta의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    total_agent = gameState.getNumAgents()
    evaluation = self.evaluationFunction
    depth_limit = self.depth
    alpha = float("-inf") ;beta = float("inf")
    def terminal_state(S,D):
      if S.isWin() or S.isLose():
        return True
      else:
        return False
    def pacman_turn(depth,current,agent,a,b):
      if not terminal_state(current, depth):
        candidate = []
        v = float("-inf")
        for move in current.getLegalActions(agent):
          following = current.generateSuccessor(agent,move)
          value = ghost_turn(depth, following, agent + 1,a,b)
          heappush(candidate, [(-1) * value, move])
          if value > v:
            a = max(a, value)
            v = value
          if v > b: ##pruning
            return heappop(candidate)[0]*(-1)

        if depth == 1:
          return heappop(candidate)[1]
        return heappop(candidate)[0]*(-1)
      else:
        return evaluation(current)

    def ghost_turn(depth,current,agent,a,b):
      if not terminal_state(current, depth):
        candidate = []
        v = float("inf")
        for move in current.getLegalActions(agent):
          following = current.generateSuccessor(agent, move)
          if agent + 1 == total_agent:
            if depth < depth_limit:
              value = pacman_turn(depth+1,following,0,a,b)
              heappush(candidate,[value,move])
            else:
              value = evaluation(following)
              heappush(candidate, [value, move])
          else:
            value = ghost_turn(depth, following, agent + 1,a,b)
            heappush(candidate,[value,move])
          if value < v:
            b = min(b, value)
            v = value
          if v < a: ##pruning
            return heappop(candidate)[0]
        return heappop(candidate)[0]

      else:
        return evaluation(current)

    return pacman_turn(1,gameState,0,alpha,beta)

    #raise Exception("Not implemented yet")

    ############################################################################



class ExpectimaxAgent(AdversialSearchAgent):
  """
    [문제 03] Expectimax의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    total_agent = gameState.getNumAgents()
    evaluation = self.evaluationFunction
    depth_limit = self.depth

    def terminal_state(S, D):
      if S.isWin() or S.isLose():
        return True
      else:
        return False

    def pacman_turn(depth, current, agent):
      if not terminal_state(current, depth):
        moving = current.getLegalActions(agent)
        candidate = []
        for move in moving:
          following = current.generateSuccessor(agent, move)
          heappush(candidate, [(-1) * ghost_turn(depth, following, agent + 1), move])
        if depth == 1:
          return heappop(candidate)[1]
        return heappop(candidate)[0] * (-1)
      else:
        return evaluation(current)

    def ghost_turn(depth, current, agent):
      if not terminal_state(current, depth):
        moving = current.getLegalActions(agent)
        candidate = []
        exp = 1.0 / float(len(moving))
        value = 0
        for move in moving:
          following = current.generateSuccessor(agent, move)
          if agent + 1 == total_agent:
            if depth < depth_limit:
              value += pacman_turn(depth + 1, following, 0)
            else:
              value += evaluation(following)
          else:
            value += ghost_turn(depth, following, agent + 1)
        return float(value) / exp

      else:
        return evaluation(current)

    return pacman_turn(1, gameState, 0)

    raise Exception("Not implemented yet")

    ############################################################################
