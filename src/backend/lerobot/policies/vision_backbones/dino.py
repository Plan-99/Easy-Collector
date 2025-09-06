import torch
import torch.nn as nn
from transformers import AutoModel

class DinoBackbone(nn.Module):
    def __init__(self, model_name='facebook/dinov2-base', output_channels=512, spatial_size=7):
        super().__init__()
        # 1. DINOv2 모델 로드
        self.dinov2 = AutoModel.from_pretrained(model_name)
        
        dino_output_dim = self.dinov2.config.hidden_size # DINOv2-Base의 경우 768
        
        # 2. 채널 수를 맞추기 위한 프로젝션 레이어 (768 -> 512)
        #    이 레이어가 이전 코드의 encoder_img_feat_input_proj 역할을 대신하게 됩니다.
        self.projection = nn.Conv2d(dino_output_dim, output_channels, kernel_size=1)
        
        # 3. 불안정한 reshape를 대체할 AdaptiveAvgPool2d
        #    어떤 개수의 패치가 들어와도 (spatial_size x spatial_size) 크기로 맞춰줍니다.
        self.pool = nn.AdaptiveAvgPool2d((spatial_size, spatial_size))
        
        # 4. 최종 출력 채널 수를 num_channels 속성으로 저장
        self.num_channels = output_channels # ResNet과 동일한 512
        
        # 호환성을 위한 더미 fc 레이어
        self.fc = nn.Linear(self.num_channels, 1)

    def forward(self, x: torch.Tensor):
        # DINOv2 특징 추출
        outputs = self.dinov2(pixel_values=x)
        patch_tokens = outputs.last_hidden_state[:, 1:, :] # [B, N, D]
        
        # [B, N, D] -> [B, D, N]
        patch_tokens = patch_tokens.permute(0, 2, 1)
        
        # 임시 2D 형태로 변환: [B, D, N] -> [B, D, N, 1]
        # AdaptiveAvgPool2d가 4D 입력을 기대하기 때문
        b, d, n = patch_tokens.shape
        # N이 HxW 역할을 하도록 임시 reshape
        # N의 제곱근을 계산할 필요 없이, 가로가 N, 세로가 1인 형태로 만듦
        temp_feature_map = patch_tokens.reshape(b, d, n, 1)

        # 고정된 크기의 2D 특징 맵으로 풀링: [B, D, N, 1] -> [B, D, H_out, W_out]
        pooled_feature_map = self.pool(temp_feature_map)
        
        # 채널 수 프로젝션: [B, 768, 7, 7] -> [B, 512, 7, 7]
        final_feature_map = self.projection(pooled_feature_map)
        
        # 최종 출력을 딕셔너리 형태로 반환
        return {"feature_map": final_feature_map}