# Reference https://towardsdatascience.com/step-by-step-r-cnn-implementation-from-scratch-in-python-e97101ccde55
# For annotating https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.html#annotating-images
# Ref, recommended by david https://towardsdatascience.com/understanding-neural-networks-19020b758230


from __future__ import absolute_import, division, print_function, unicode_literals
import os
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf


BATCH_SIZE = 4
EPOCHS = 15
IMG_HEIGHT = 150
IMG_WIDTH = 150


PATH = os.path.join(os.path.curdir, 'datasets')  # Path to the dataset
train_dir = os.path.join(PATH, 'train')  # Path to training data
validation_dir = os.path.join(PATH, 'validation')

# Training sets
train_bottle_dir = os.path.join(train_dir, 'bottle')
train_rubber_duck_dir = os.path.join(train_dir, 'rubber_duck')

# Validation sets
validation_bottle_dir = os.path.join(validation_dir, 'bottle')
validation_rubber_duck_dir = os.path.join(validation_dir, 'rubber_duck')

num_bottles_train = len(os.listdir(train_bottle_dir))
num_ducks_train = len(os.listdir(train_rubber_duck_dir))

num_bottles_val = len(os.listdir(validation_bottle_dir))
num_ducks_val = len(os.listdir(validation_rubber_duck_dir))


total_train_data = num_bottles_train + num_ducks_train
total_val_data = num_bottles_val + num_ducks_val


train_image_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1./255)  # Generator for training data
validation_image_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1./255)  # Generator for validation data

# Load training images, rescale them according to what we just defined
train_data_gen = train_image_generator.flow_from_directory(batch_size=BATCH_SIZE,
                                                           directory=train_dir,
                                                           shuffle=True,
                                                           target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                           class_mode='binary')


# Load the validation images and process them as specified before
val_data_gen = validation_image_generator.flow_from_directory(batch_size=BATCH_SIZE,
                                                              directory=validation_dir,
                                                              target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                              class_mode='binary')


# Create the training model, literally just stacking layers on layers with relu or sigmoid activation function
model = tf.keras.Sequential([tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu', input_shape=(IMG_HEIGHT, IMG_WIDTH, 3)),
                             tf.keras.layers.MaxPool2D(),
                             tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu'),
                             tf.keras.layers.MaxPool2D(),
                             tf.keras.layers.Conv2D(64, 3, padding='same', activation='relu'),
                             tf.keras.layers.MaxPool2D(),
                             tf.keras.layers.Flatten(),
                             tf.keras.layers.Dense(512, activation='relu'),
                             tf.keras.layers.Dense(1, activation='sigmoid')
                             ])

# Compile the model that was just created
model.compile(optimizer='adam',
              loss='binary_crossentropy',
              metrics=['accuracy'])

# model.summary()

# Train the model we just created
history = model.fit_generator(train_data_gen,
                              steps_per_epoch=total_train_data // BATCH_SIZE,
                              epochs=EPOCHS,
                              validation_data=val_data_gen,
                              validation_steps=total_val_data // BATCH_SIZE
                              )



