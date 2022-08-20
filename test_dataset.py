import pickle

with open('dataset/test_traj10000/test.pkl', 'rb') as f:
    loaded_data = pickle.load(f)

for demo in loaded_data:
    for key, value in demo.items():
        print(key)
        print(value.shape)
        print(type(value))
print(len(loaded_data))
