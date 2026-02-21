## 1. NVIDIA Container Toolkit Kurulumu

Docker'ın sisteminizdeki ekran kartını tanıyıp kullanabilmesi için aşağıdaki komutları sırasıyla terminalde çalıştırın:

```bash
# Depo anahtarlarını sisteme ekleme
curl -fsSL [https://nvidia.github.io/libnvidia-container/gpgkey](https://nvidia.github.io/libnvidia-container/gpgkey) | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L [https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list](https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list) | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Paket listesini güncelleyip aracı kurma
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
