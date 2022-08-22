import pickle
import cv2
import matplotlib.pyplot as plt
import numpy as np

def check_data_len(file_path='../dataset/test.pkl'):
    with open(file_path, 'rb') as f:
        loaded_data = pickle.load(f)

    for demo in loaded_data:
        for key, value in demo.items():
            print(key)
            print(value.shape)
            print(type(value))
    print(len(loaded_data))


def show_image(image):
    img = (image.to('cpu').detach().numpy().transpose(0, 1, 2)).astype(np.uint8).copy()   
    # img = cv2.resize(img, (w, h))
    # plt.axis('off')
    # plt.imshow(img)
    # plt.show()
    cv2.imwrite('test_2.jpg', img)

# visualize image
def visualize_image(file_path='../dataset/test_1.pkl'):
    with open(file_path, 'rb') as f:
        loaded_data = pickle.load(f)
    test_image = loaded_data[0]['images'][38]
    print(len(loaded_data[0]['images']))
    print(test_image.shape)
    show_image(test_image)


if __name__=='__main__':
    visualize_image()

