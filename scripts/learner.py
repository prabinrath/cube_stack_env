import torch
import torch.nn as nn
import torch.nn.functional as F

class CubeStackRGBDPolicy(nn.Module):
    def __init__(self, in_channels, act_dim):
        super().__init__()

        # CNN Layers
        self.cnn1 = nn.Conv2d(in_channels=in_channels, out_channels=32, kernel_size=8, stride=4)
        h, w = self.calc_conv2d_output_dim(in_dim = (224,224), kernel_size=(8,8), stride=(4,4))
        self.cnn2 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=4, stride=2)
        h, w = self.calc_conv2d_output_dim(in_dim = (h,w), kernel_size=(4,4), stride=(2,2))
        self.cnn3 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1)
        h, w = self.calc_conv2d_output_dim(in_dim = (h,w), kernel_size=(3,3), stride=(1,1))

        # Policy FCC
        self.fcc1 = nn.Linear(h*w*64, 512)
        self.fcc2 = nn.Linear(512, act_dim)

    def calc_conv2d_output_dim(self, in_dim, kernel_size, padding=(0,0), dialation=(1,1), stride=(1,1)):
        h = (in_dim[0]+2*padding[0]-dialation[0]*(kernel_size[0]-1)-1)//stride[0]+1
        w = (in_dim[1]+2*padding[1]-dialation[1]*(kernel_size[1]-1)-1)//stride[1]+1
        return h, w
    
    def init_weights(self, m):
        if type(m) == nn.Linear:
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')    
            torch.nn.init.constant_(m.bias, 0) 
        if type(m) == nn.Conv2d:
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            torch.nn.init.constant_(m.bias, 0)

    def forward(self, x):
        x = F.relu(self.cnn1(x))
        x = F.relu(self.cnn2(x))
        x = F.relu(self.cnn3(x))
        x = x.view(x.size(0),-1)

        a = F.relu(self.fcc1(x))
        a = self.fcc2(a)
        return a

class CubeStackRGBDValue(nn.Module):
    def __init__(self, in_channels):
        super().__init__()

        # CNN Layers
        self.cnn1 = nn.Conv2d(in_channels=in_channels, out_channels=32, kernel_size=8, stride=4)
        h, w = self.calc_conv2d_output_dim(in_dim = (224,224), kernel_size=(8,8), stride=(4,4))
        self.cnn2 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=4, stride=2)
        h, w = self.calc_conv2d_output_dim(in_dim = (h,w), kernel_size=(4,4), stride=(2,2))
        self.cnn3 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1)
        h, w = self.calc_conv2d_output_dim(in_dim = (h,w), kernel_size=(3,3), stride=(1,1))

        # Policy FCC
        self.fcc1 = nn.Linear(h*w*64, 256)
        self.fcc2 = nn.Linear(256, 1)

    def calc_conv2d_output_dim(self, in_dim, kernel_size, padding=(0,0), dialation=(1,1), stride=(1,1)):
        h = (in_dim[0]+2*padding[0]-dialation[0]*(kernel_size[0]-1)-1)//stride[0]+1
        w = (in_dim[1]+2*padding[1]-dialation[1]*(kernel_size[1]-1)-1)//stride[1]+1
        return h, w
    
    def init_weights(self, m):
        if type(m) == nn.Linear:
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')    
            torch.nn.init.constant_(m.bias, 0) 
        if type(m) == nn.Conv2d:
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            torch.nn.init.constant_(m.bias, 0)

    def forward(self, x):
        x = F.relu(self.cnn1(x))
        x = F.relu(self.cnn2(x))
        x = F.relu(self.cnn3(x))
        x = x.view(x.size(0),-1)

        a = F.relu(self.fcc1(x))
        a = self.fcc2(a)
        return a