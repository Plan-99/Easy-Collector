import torch.nn as nn
from torch.nn import functional as F
import torchvision.transforms as transforms

from .act.detr.main import build_ACT_model_and_optimizer, build_CNNMLP_model_and_optimizer

from pytorch_grad_cam import EigenCAM
from pytorch_grad_cam.utils.image import show_cam_on_image, preprocess_image
import torch
import cv2
import numpy as np

import IPython

e = IPython.embed

class ACTPolicy(nn.Module):
    def __init__(self, args_override):
        super().__init__()
        model, optimizer = build_ACT_model_and_optimizer(args_override)
        self.model = model # CVAE decoder
        self.optimizer = optimizer
        self.kl_weight = args_override['kl_weight']
        print(f'KL Weight {self.kl_weight}')

    def __call__(self, qpos, image, actions=None, is_pad=None):
        
        env_state = None
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
        image = self.normalize(image)
        if actions is not None: # training time
            actions = actions[:, :self.model.num_queries]
            is_pad = is_pad[:, :self.model.num_queries]
            a_hat, is_pad_hat, (mu, logvar) = self.model(qpos, image, env_state, actions, is_pad)
            total_kld, dim_wise_kld, mean_kld = kl_divergence(mu, logvar)
            loss_dict = dict()
            all_l1 = F.l1_loss(actions, a_hat, reduction='none')
            l1 = (all_l1 * ~is_pad.unsqueeze(-1)).mean()
            loss_dict['l1'] = l1
            loss_dict['kl'] = total_kld[0]
            loss_dict['loss'] = loss_dict['l1'] + loss_dict['kl'] * self.kl_weight
            return loss_dict
        else: # inference time
            a_hat, _, (_, _) = self.model(qpos, image, env_state) # no action, sample from prior
            return a_hat

    def configure_optimizers(self):
        return self.optimizer
    
    def show_grad_cam_heatmap(self, image_tensor, cam_len):
        backbone_model = self.model.backbones[0][0].body
        target_layer = backbone_model.layer4[-1]
        wrapped_model = WrapperModel(backbone_model)

        visualizations = []
        for cam_id in range(cam_len):
            image = image_tensor[:, cam_id]
            norm_image = self.normalize(image)
            

            cam = EigenCAM(model=wrapped_model, target_layers=[target_layer])
            # heatmaps.append(cam(image)[0])
            heatmap = cam(image)[0]
            image = image.cpu()[0].numpy().transpose(1, 2, 0)
            visualizations.append(show_cam_on_image(image, heatmap, use_rgb=True))

        view = np.concatenate(visualizations, axis=1)
        # 5️⃣ cv2를 사용한 시각화
        visualization_bgr = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)  # OpenCV는 BGR 형식을 사용하므로 변환 필요
        resized = cv2.resize(visualization_bgr, (640*3, 480))
        
        cv2.imshow("eigencam_result", resized)
        cv2.waitKey(1)
        
class WrapperModel(nn.Module):
    def __init__(self, base_model):
        super().__init__()
        self.base_model = base_model

    def forward(self, x):
        out = self.base_model(x)  # OrderedDict 반환
        return list(out.values())[0]  # 첫 번째 Tensor만 반환



# class GradCAM:
#     def __init__(self, model, target_layer):
#         self.model = model
#         self.target_layer = target_layer
#         self.gradients = None
#         self.activations = None
#         self.hook_layers()

#     def hook_layers(self):
#         def forward_hook(module, input, output):
#             self.activations = output
#         def backward_hook(module, grad_in, grad_out):
#             self.gradients = grad_out[0]

#         self.target_layer.register_forward_hook(forward_hook)
#         self.target_layer.register_backward_hook(backward_hook)

#     def generate_heatmap_mean(self, input_tensor, feature_dim=0):
#         output = self.model(input_tensor)['0']  # ResNet18 Backbone Forward
#         loss = output[:, feature_dim, :, :].mean()
#         self.model.zero_grad()
#         loss.requires_grad_()
#         loss.backward()

#         print(loss)
#         quit()
#         weights = torch.mean(self.gradients, dim=[2, 3], keepdim=True)
#         cam = torch.sum(weights * self.activations, dim=1)
#         cam = F.relu(cam)
#         cam = cam.squeeze().cpu().detach().numpy()

#         cam = cv2.resize(cam, (input_tensor.shape[2], input_tensor.shape[3]))
#         cam = (cam - np.min(cam)) / (np.max(cam) - np.min(cam))
#         return cam
    



class CNNMLPPolicy(nn.Module):
    def __init__(self, args_override):
        super().__init__()
        model, optimizer = build_CNNMLP_model_and_optimizer(args_override)
        self.model = model # decoder
        self.optimizer = optimizer

    def __call__(self, qpos, image, actions=None, is_pad=None):
        env_state = None # TODO
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
        image = normalize(image)
        if actions is not None: # training time
            actions = actions[:, 0]
            a_hat = self.model(qpos, image, env_state, actions)
            mse = F.mse_loss(actions, a_hat)
            loss_dict = dict()
            loss_dict['mse'] = mse
            loss_dict['loss'] = loss_dict['mse']
            return loss_dict
        else: # inference time
            a_hat = self.model(qpos, image, env_state) # no action, sample from prior
            return a_hat

    def configure_optimizers(self):
        return self.optimizer

def kl_divergence(mu, logvar):
    batch_size = mu.size(0)
    assert batch_size != 0
    if mu.data.ndimension() == 4:
        mu = mu.view(mu.size(0), mu.size(1))
    if logvar.data.ndimension() == 4:
        logvar = logvar.view(logvar.size(0), logvar.size(1))

    klds = -0.5 * (1 + logvar - mu.pow(2) - logvar.exp())
    total_kld = klds.sum(1).mean(0, True)
    dimension_wise_kld = klds.mean(0)
    mean_kld = klds.mean(1).mean(0, True)

    return total_kld, dimension_wise_kld, mean_kld
