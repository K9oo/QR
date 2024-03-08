# QR code를 필요에 맞게 생성 가능한 코드
# 'data'와 'img.save'의 이름을 바꿔서 실행
import qrcode

qr = qrcode.QRCode(
        version = 1,
        box_size = 15,
        border = 5
)

data = 'CJU4090404'
qr.add_data(data)
qr.make(fit=True)
img = qr.make_image(fill='black', black_color = 'white')
img.save('CJU4090404.png')