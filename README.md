# Sadit Bot - Docker Kurulum ve Kullanım Rehberi (NVIDIA & Ubuntu 22.04)

Bu rehber, **Sadit Bot** projesini Ubuntu 22.04 üzerinde NVIDIA ekran kartı (GPU) hızlandırması kullanarak çalıştırmak için gereken tüm adımları içerir. Bu sayede Gazebo, RViz gibi grafiksel arayüz (GUI) gerektiren araçları yüksek performansla kullanabilirsiniz.

## 🛠 Ön Koşullar

Başlamadan önce sisteminizde şunların kurulu olduğundan emin olun:
1.  **NVIDIA Sürücüleri:** (Terminalde `nvidia-smi` yazarak kontrol edebilirsiniz).
2.  **Docker ve Docker Compose:**

---

## 🚀 Kurulum Adımları

### Adım 1: NVIDIA Container Toolkit Kurulumu
Docker'ın sisteminizdeki ekran kartını tanıyabilmesi için bu aracı kurmanız zorunludur. Terminali açın ve sırasıyla şu komutları çalıştırın:

```bash
# 1. Depo anahtarlarını sisteme ekleyin
curl -fsSL [https://nvidia.github.io/libnvidia-container/gpgkey](https://nvidia.github.io/libnvidia-container/gpgkey) | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L [https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list](https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list) | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Paket listesini güncelleyin ve aracı kurun
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3. Docker'a aracı tanıtın ve servisi yeniden başlatın
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
