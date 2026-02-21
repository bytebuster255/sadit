## 🛠 1. İlk Kurulum (Sadece Bir Kere Yapılacak)

Docker'ın sisteminizdeki NVIDIA ekran kartını donanım seviyesinde tanıyabilmesi için "NVIDIA Container Toolkit" kurmanız zorunludur. 

Terminali açın ve sırasıyla şu komutları çalıştırın:

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
```

---

## ⚙️ 2. Konfigürasyon (`docker-compose.yml` Ayarı)

Proje dizininde bulunan `docker-compose.yml` dosyanızı açın. NVIDIA kullanan bilgisayarlar için ilgili satırların başındaki `#` (yorum) işaretlerini silerek aktif hale getirin. 

Dosyanızın o kısımları **tam olarak** şöyle görünmelidir:

```yaml
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - MESA_GL_VERSION_OVERRIDE=3.3
      - GAZEBO_MODEL_DATABASE_URI=
      # NVIDIA İÇİN ÇEVRE DEĞİŞKENLERİ AKTİF EDİLDİ
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.gazebo/models:/root/.gazebo/models
    devices:
      - /dev/dri:/dev/dri
    
    # NVIDIA EKRAN KARTI AYARI AKTİF EDİLDİ
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

---



## 🚀 Nasıl Çalıştırılır? 

İlk kurulum adımlarını (aşağıdaki 1. ve 2. bölümler) bir kez tamamladıktan sonra, projeyi her çalıştırmak istediğinizde **sadece bu adımları** izlemeniz yeterlidir:

**Adım 1:** Terminali açın ve `docker-compose.yml` dosyasının bulunduğu proje klasörüne gidin.

**Adım 2:** Arayüz (GUI) uygulamalarının bilgisayarınızın ekranına yansıması için gerekli X11 iznini verin:
```bash
xhost +local:root
```
*(Not: Bu komut bilgisayarı her yeniden başlattığınızda sıfırlanır, bu yüzden konteyneri başlatmadan önce girilmesi önemlidir.)*

**Adım 3:** Konteyneri arka planda başlatın (Eğer Dockerfile'da veya docker-compose.yml dosyasında bir değişiklik yaptıysanız komuta `--build` parametresini eklemeyi unutmayın):
```bash
docker compose up -d
```

**Adım 4:** Çalışan konteynerin içine girin ve terminalini açın:
```bash
docker exec -it sadit_container bash
```

**Adım 5:** Artık konteynerin içindesiniz (`root@sadit_container:/#`). Burada simülasyonlarınızı ve ROS 2 düğümlerinizi çalıştırabilirsiniz. Örnek komutlar:
```bash
# Gazebo simülasyonunu başlatmak için:
gazebo

# Veya ROS 2 projenizi başlatmak için:
# source /opt/ros/<sürüm>/setup.bash
# ros2 launch <paket_adınız> <launch_dosyanız.py>
```

---

## 🛑 Sistemi Kapatmak

İşiniz bittiğinde konteynerden çıkmak ve arka planda çalışan sistemi tamamen durdurmak için şu adımları izleyin:

1. Konteyner terminalinden çıkın:
   ```bash
   exit
   ```
2. Proje dizininde (konteynerin dışında) sistemi kapatın:
   ```bash
   docker compose down
   ```

---
---


## ⚠️ Sık Karşılaşılan Sorunlar (Sorun Giderme)

* **"Cannot open display" veya GUI Açılmama Hatası:** `xhost +local:root` komutunu konteyneri başlatmadan *önce* kendi terminalinizde çalıştırdığınızdan emin olun. Eğer Ubuntu'da Wayland kullanıyorsanız (varsayılan olabilir), oturumu kapatıp giriş ekranında sağ alttaki çark ikonundan "Ubuntu on Xorg" seçeneği ile giriş yapmayı deneyin.
* **Konteyner içindeyken Gazebo çok yavaş:** NVIDIA Container Toolkit'in doğru kurulduğundan ve `docker-compose.yml` ayarlarının aktif olduğundan emin olun. Konteyner içindeyken terminale `nvidia-smi` yazarak ekran kartınızın konteyner tarafından görülüp görülmediğini test edebilirsiniz.
* **"Permission denied" Hatası (Docker çalıştırırken):** Komutların başına `sudo` ekleyin veya kullanıcınızı docker grubuna kalıcı olarak ekleyin (`sudo usermod -aG docker $USER` yapıp bilgisayarı yeniden başlatın).
