import torch
from torch import nn
from torch.distributions.normal import Normal
import torch.nn.functional as F
import math

if torch.cuda.is_available():
    device = torch.device('cuda')
else:
    device = torch.device('cpu')
print(device)


class ModuleConfig:
    updatePeriod = 400
    inputTensor = None
    actCoords = None
    training = False
    maxPos = None
    centerPos = None
    theta = None
    steps = 0
    actBits = 8

    batchRewards = []
    batchObservations = []
    batchZ = []

    def __init__(self, updatePeriod=None, inputTensor=None, actCoords=None):
        if updatePeriod is not None:
            self.updatePeriod = updatePeriod
        if inputTensor is not None:
            self.inputTensor = inputTensor
        if actCoords is not None:
            self.actCoords = actCoords


uasisiConfig = ModuleConfig()

def interp(xNew, x, y):
    idx = torch.searchsorted(x, xNew)
    lIdx = torch.clamp(idx - 1, 0, len(x) - 2)
    rIdx = torch.clamp(idx, 1, len(x) - 1)
    x0 = x[lIdx]
    x1 = x[rIdx]
    y0 = y[lIdx]
    y1 = y[rIdx]
    m = (xNew - x0)/(x1 - x0 + 1e-10)
    return y0 + m*(y1 - y0)

def dictToTorch(data):  # Meant to run on each signal dict
    if not (('data' in data) and ('type' in data)):
        raise RuntimeError("Invalid signal dict structure")
    if (data['type'] == 'DOB') and ('coords' in data):
        dataTensor = torch.tensor(data['data'])
        coordsTensor = torch.tensor(data['coords'])
    elif (data['type'] == 'VEC') and ('coords' in data):
        dataTensor = torch.tensor(data['data'])
        coordsTensor = torch.tensor(data['coords'])
    elif (data['type'] == 'SCA') and isinstance(data['data'], (int, float)):
        return data['data']  # Not torch, just value
    else:
        raise RuntimeError("Unrecognized or unsupported signal type")

    return coordsTensor, dataTensor


def computeReward(currError, prevError):
    totalCurrError = torch.trapezoid(currError)
    reward = -totalCurrError
    return totalCurrError, reward


def rewardToGo(rewards, r=0.99):
    rewards = torch.tensor(rewards)
    n = len(rewards)
    rewardsTG = torch.zeros(n)
    for i in reversed(range(n)):
        rewardsTG[i] = rewards[i] + (r*rewardsTG[i+1] if i+1 < n else 0)
    return rewardsTG


def torchToDict(data, coords=None):
    if (coords is None) and (isinstance(data, (int, float))):
        outputDict = {}
        outputDict["type"] = "SCA"
        outputDict["data"] = data
    elif isinstance(data, torch.Tensor) and isinstance(coords, torch.Tensor):
        if (data.dim() == 2):
            if not (data.shape[0] == coords.shape[0]):
                raise RuntimeError("Size mismatch")
            outputDict = {}
            outputDict["type"] = "VEC"
            outputDict["data"] = data.tolist()
            outputDict["coords"] = coords.tolist()
        elif (data.dim() == 1):
            if not (data.shape[0] == coords.shape[0]):
                raise RuntimeError("Size mismatch")
            outputDict = {}
            outputDict["type"] = "DOB"
            outputDict["data"] = data.tolist()
            outputDict["coords"] = coords.tolist()
        else:
            raise RuntimeError("Invalid tensor shape")
    else:
        raise RuntimeError("Invalid data")
    return outputDict


class ctrlCNN(nn.Module):
    def __init__(self, nPoints, nActuators, conv1Ch=8, kSize=5):
        if (not isinstance(nPoints, int)) or (not isinstance(nActuators, int)):
            raise RuntimeError("Invalid number of points or actuators")
        if (nPoints < 2*nActuators) or (nActuators < 0) or (nPoints < 0):
            raise RuntimeError("Cannot create network, invalid sizes")
        if (conv1Ch < 4):
            raise RuntimeError("conv1Ch must be at least 4")
        super().__init__()

        self.nPts = nPoints
        self.nActs = nActuators
        self.maxDs = math.floor(math.log2(self.nPts/self.nActs))
        self.dsPts = math.ceil(self.nPts/2**self.maxDs)
        self.inCh = conv1Ch
        self.kSz = kSize
        self.pdd = round((self.kSz - 1) / 2)

        self.conv1 = nn.Conv1d(4, self.inCh, kernel_size=self.kSz, padding=self.pdd)

        self.convds = nn.ModuleList()
        for _ in range(self.maxDs):
            self.outCh = 2*self.inCh
            self.convds.append(
                nn.Conv1d(self.inCh, self.outCh, kernel_size=self.kSz, stride=2, padding=self.pdd)
            )
            self.inCh = self.outCh

        self.conv2 = nn.Conv1d(self.inCh, self.inCh, kernel_size=self.kSz, padding=self.pdd)

        self.pool = nn.AdaptiveAvgPool1d(self.nActs)

        self.outCh = round(math.sqrt(self.inCh))
        self.perActuator = nn.ModuleList([  # Flatten with per-actuator weights
            nn.Sequential(
                nn.Linear(self.inCh, self.outCh),
                nn.ReLU(),
                nn.Linear(self.outCh, 1)
            ) for _ in range(self.nActs)
        ])

        self.fc = nn.Sequential(
            nn.Linear(self.nActs, 2*self.nActs),
            nn.ReLU(),
            nn.Linear(2*self.nActs, self.nActs)
        )

        self.logSigma = nn.Parameter(torch.zeros(self.nActs))

    def forward(self, x):
        x = F.relu(self.conv1(x))
        for layer in self.convds:
            x = F.relu(layer(x))
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = x.transpose(1, 2)
        y = []
        for i in range(self.nActs):
            yAct = self.perActuator[i](x[:, i, :])
            y.append(yAct)
        y = torch.cat(y, dim=1)
        mu = self.fc(y)
        sigma = self.logSigma.exp()
        return Normal(mu, sigma)

    def getAction(self, obs, deterministic=False):
        if deterministic:
            z = self(obs).mean
        else:
            z = self(obs).sample().detach()
        action = z / (1 + z.abs())
        threshold = 1 - (1 / 2**(uasisiConfig.actBits - 1))
        action[action > threshold] = 1.0
        action[action < -threshold] = -1.0
        action *= uasisiConfig.maxPos
        action += uasisiConfig.centerPos
        return z, action

    def computeLogP(self, obs, z):
        dist = self(obs)
        return dist.log_prob(z).sum(dim=-1)


def declareSignals():
    return [
        {'name': 'targetLift', 'dType': 'DOB', 'sType': 'IN'},
        {'name': 'realLift', 'dType': 'DOB', 'sType': 'IN'},
        {'name': 'realGeometry', 'dType': 'DOB', 'sType': 'IN'},
        {'name': 'targetGeometry', 'dType': 'DOB', 'sType': 'OUT'},
        {'name': 'realError', 'dType': 'VEC', 'sType': 'OUT'},  # For vis
        {'name': 'totalError', 'dType': 'SCA', 'sType': 'OUT'},
        {'name': 'reward', 'dType': 'SCA', 'sType': 'OUT'}
    ]


def init(inputs, model, optimizer):

    outputs = {}
    if uasisiConfig.actCoords is None:
        raise RuntimeError("Actuator coordinates must be set")
    if uasisiConfig.maxPos is None:
        raise RuntimeError("Max actuator position must be set")
    if uasisiConfig.centerPos is None:
        raise RuntimeError("Actuator center points must be set")
    if uasisiConfig.theta is None:
        raise RuntimeError("Actuator initial position must be set")
    if not (uasisiConfig.actCoords.shape == uasisiConfig.maxPos.shape == uasisiConfig.centerPos.shape == uasisiConfig.theta.shape):
        raise RuntimeError("Size mismatch")
    if 'DOB' not in inputs:
        raise RuntimeError("Invalid input dict structure")
    if not ('targetLift' in inputs['DOB'] and
            'realLift' in inputs['DOB'] and
            'realGeometry' in inputs['DOB']):
        raise RuntimeError("Missing signals")

    pts = len(inputs['DOB']['targetLift']['coords'])
    uasisiConfig.inputTensor = torch.zeros(1, 4, pts)

    model.to(device)

    def initTrain():

        model.train()
        coords, tLift = dictToTorch(inputs['DOB']['targetLift'])
        rLift = torch.zeros_like(tLift)
        uasisiConfig.inputTensor[0, 0, :] = tLift - rLift
        uasisiConfig.inputTensor[0, 1, :] = interp(coords, uasisiConfig.actCoords, uasisiConfig.theta)
        uasisiConfig.inputTensor[0, 2, :] = tLift - rLift
        uasisiConfig.inputTensor[0, 3, :] = uasisiConfig.inputTensor[0, 1, :]
        tError, reward = computeReward(uasisiConfig.inputTensor[0, 2, :], uasisiConfig.inputTensor[0, 0, :])

        uasisiConfig.batchObservations = []
        uasisiConfig.batchRewards = []
        uasisiConfig.batchZ = []

        uasisiConfig.inputTensor = uasisiConfig.inputTensor.to(device)
        y = interp(uasisiConfig.actCoords, coords, uasisiConfig.inputTensor[0, 3, :])
        rError = torch.stack([tLift, rLift, uasisiConfig.inputTensor[0, 2, :]], dim=1)
        outputs['DOB'] = {}
        outputs['VEC'] = {}
        outputs['SCA'] = {}
        outputs['DOB']['targetGeometry'] = torchToDict(data=y, coords=uasisiConfig.actCoords)
        outputs['VEC']['realError'] = torchToDict(data=rError, coords=coords)
        outputs['SCA']['totalError'] = torchToDict(data=tError.item())
        outputs['SCA']['reward'] = torchToDict(data=reward.item())
        return outputs

    def initTest():
        
        outputs = {}
        model.eval()
        coords, tLift = dictToTorch(inputs['DOB']['targetLift'])
        rLift = torch.zeros_like(tLift)
        uasisiConfig.inputTensor[0, 0, :] = tLift - rLift
        uasisiConfig.inputTensor[0, 1, :] = interp(coords, uasisiConfig.actCoords, uasisiConfig.theta)
        uasisiConfig.inputTensor[0, 2, :] = tLift - rLift
        uasisiConfig.inputTensor[0, 3, :] = uasisiConfig.inputTensor[0, 1, :]
        tError, reward = computeReward(uasisiConfig.inputTensor[0, 2, :], uasisiConfig.inputTensor[0, 0, :])

        uasisiConfig.inputTensor = uasisiConfig.inputTensor.to(device)
        y = interp(uasisiConfig.actCoords, coords, uasisiConfig.inputTensor[0, 3, :])
        rError = torch.stack([tLift, rLift, uasisiConfig.inputTensor[0, 2, :]], dim=1)
        outputs['DOB'] = {}
        outputs['VEC'] = {}
        outputs['SCA'] = {}
        outputs['DOB']['targetGeometry'] = torchToDict(data=y, coords=uasisiConfig.actCoords)
        outputs['VEC']['realError'] = torchToDict(data=rError, coords=coords)
        outputs['SCA']['totalError'] = torchToDict(data=tError.item())
        outputs['SCA']['reward'] = torchToDict(data=reward.item())
        return outputs

    if uasisiConfig.training:
        outputs = initTrain()
    else:
        outputs = initTest()
    uasisiConfig.steps += 1
    return outputs


def step(inputs, t, dt, model, optimizer, mode=None):

    if 'DOB' not in inputs:
        raise RuntimeError("Invalid input dict structure")
    if not ('targetLift' in inputs['DOB'] and
            'realLift' in inputs['DOB'] and
            'realGeometry' in inputs['DOB']):
        raise RuntimeError("Missing signals")

    def getLoss(obs, z, weights):
        logP = model.computeLogP(obs, z)
        return -(logP * weights).mean()

    def stepTrain():

        outputs = {}
        uasisiConfig.inputTensor[0, 0, :] = uasisiConfig.inputTensor[0, 2, :]  # Previous error
        uasisiConfig.inputTensor[0, 1, :] = uasisiConfig.inputTensor[0, 3, :]  # Previous position
        coords, tLift = dictToTorch(inputs['DOB']['targetLift'])
        _, rLift = dictToTorch(inputs['DOB']['realLift'])
        uasisiConfig.inputTensor[0, 2, :] = tLift - rLift
        _, uasisiConfig.inputTensor[0, 3, :] = dictToTorch(inputs['DOB']['realGeometry'])
        tError, reward = computeReward(uasisiConfig.inputTensor[0, 2, :], uasisiConfig.inputTensor[0, 0, :])

        if (uasisiConfig.steps % uasisiConfig.updatePeriod == 0):
            batchObs = torch.cat(uasisiConfig.batchObservations, dim=0)
            batchZ = torch.cat(uasisiConfig.batchZ, dim=0)
            batchWeights = rewardToGo(uasisiConfig.batchRewards)
            batchWeights = (batchWeights - batchWeights.mean()) / (batchWeights.std() + 1e-8)
            batchWeights = batchWeights.to(device)
            optimizer.zero_grad()
            batchLoss = getLoss(batchObs, batchZ, batchWeights)
            batchLoss.backward()
            optimizer.step()
            uasisiConfig.batchObservations = []
            uasisiConfig.batchRewards = []
            uasisiConfig.batchZ = []

        uasisiConfig.inputTensor = uasisiConfig.inputTensor.to(device)
        uasisiConfig.batchObservations.append(uasisiConfig.inputTensor.clone())
        uasisiConfig.batchRewards.append(reward.item())
        z, y = model.getAction(uasisiConfig.inputTensor)
        uasisiConfig.batchZ.append(z.clone())
        rError = torch.stack([tLift, rLift, uasisiConfig.inputTensor[0, 2, :]], dim=1)
        outputs['DOB'] = {}
        outputs['VEC'] = {}
        outputs['SCA'] = {}
        outputs['DOB']['targetGeometry'] = torchToDict(data=y[0], coords=uasisiConfig.actCoords)
        outputs['VEC']['realError'] = torchToDict(data=rError, coords=coords)
        outputs['SCA']['totalError'] = torchToDict(data=tError.item())
        outputs['SCA']['reward'] = torchToDict(data=reward.item())
        return outputs

    def stepTest():

        outputs = {}
        uasisiConfig.inputTensor[0, 0, :] = uasisiConfig.inputTensor[0, 2, :]  # Previous error
        uasisiConfig.inputTensor[0, 1, :] = uasisiConfig.inputTensor[0, 3, :]  # Previous position
        coords, tLift = dictToTorch(inputs['DOB']['targetLift'])
        _, rLift = dictToTorch(inputs['DOB']['realLift'])
        uasisiConfig.inputTensor[0, 2, :] = tLift - rLift
        _, uasisiConfig.inputTensor[0, 3, :] = dictToTorch(inputs['DOB']['realGeometry'])
        tError, reward = computeReward(uasisiConfig.inputTensor[0, 2, :], uasisiConfig.inputTensor[0, 0, :])

        uasisiConfig.inputTensor = uasisiConfig.inputTensor.to(device)
        _, y = model.getAction(uasisiConfig.inputTensor, deterministic=True)
        rError = torch.stack([tLift, rLift, uasisiConfig.inputTensor[0, 2, :]], dim=1)
        outputs['DOB'] = {}
        outputs['VEC'] = {}
        outputs['SCA'] = {}
        outputs['DOB']['targetGeometry'] = torchToDict(data=y[0], coords=uasisiConfig.actCoords)
        outputs['VEC']['realError'] = torchToDict(data=rError, coords=coords)
        outputs['SCA']['totalError'] = torchToDict(data=tError.item())
        outputs['SCA']['reward'] = torchToDict(data=reward.item())
        return outputs

    if uasisiConfig.training:
        outputs = stepTrain()
    else:
        outputs = stepTest()

    uasisiConfig.steps += 1
    return outputs


def saveModel(model, optimizer, fileName):
    sd = {'model': model.state_dict(), 'opt': optimizer.state_dict()}
    torch.save(sd, fileName)


def loadModel(fileName, nPoints, nActuators, conv1Ch=8, kSize=5, lr = 0.004):
    print(fileName)
    checkpoint = torch.load(fileName)
    model = ctrlCNN(nPoints, nActuators, conv1Ch, kSize)
    model.load_state_dict(checkpoint['model'])
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    optimizer.load_state_dict(checkpoint['opt'])
    return model, optimizer
