# Mars Yard 3D Model

This file describes how to view, download, and convert the 3D models of the Mars Yard provided by the organizers.


---


## View & Download Mars Yard Model

### 2025

* **You can view the 3D model online via [this Sketchfab link](https://skfb.ly/pMyFC)**
* **You can download 3D models in different formats via [this Google Drive folder](https://drive.google.com/drive/folders/1CC6CF_olB0m846RJ8Zi-IzKdQuF7lAu2?usp=drive_link)**

### 2026

* **You can view the 3D model online via [this Sketchfab link](https://skfb.ly/pMHFs)**
* **You can download 3D models in different formats via [this Google Drive folder](https://drive.google.com/drive/folders/1nXtgAN9hhfJjxfkgsiDwHwlEYIGK4358?usp=drive_link)**


---


## Convert .e57 to Meshes

### Prerequisites: Software Installation

To process `.e57` point clouds into simulation-ready meshes, you need **CloudCompare**.

Open your terminal and install it via Snap:

```bash
sudo snap install cloudcompare
```

Once installed, you can open the software directly from the terminal by running:

```bash
cloudcompare
```

### Step 1: Open the Point Cloud

1. Open **CloudCompare**.
2. Click **File > Open** and select your `.e57` point cloud file.
3. If prompted with a *Global Shift/Scale* dialog, click **Yes to All** to load the coordinates correctly.

### Step 2: Generate Mesh using Poisson Surface Reconstruction

1. In the **DB Tree** panel on the left, click on your loaded point cloud to select it.
2. Go to the top menu and select **Plugins > PoissonRecon**.
3. In the Poisson dialog box:
* Set **Octree Depth** to **8-10** (for a high-resolution mesh) or **5-7**(for a low-resolution mesh).
* Check the box for **Output density as SF** (Scalar Field). *This is crucial for the next step to remove fake edges.*
4. Click **OK**.

### Step 3: Clear Outer Boundaries (Scalar Field Filter)

Because Poisson reconstruction tries to create a continuous surface, it generates a smooth, bubble-like outer edge where your actual data ends. You can easily cut this away using the density diagram:

1. Select your newly generated mesh in the **DB Tree**.
2. Go to the top menu and click **Edit > Scalar fields > Show histogram**.
3. A histogram diagram will pop up displaying the point density values. Note that the fake outer edges have very low density, while the actual map has high density.
4. Compare the colors on your 3D mesh to the histogram graph. Identify the numerical value that corresponds to the colors of the unwanted outer boundaries.
5. Go to the top menu and click **Edit > Scalar fields > Filter by Value**.
6. In the pop-up window, enter the value you found in Step 4 as the minimum (lower) limit. Click **Export** to generate a new, perfectly trimmed mesh in your DB Tree.
7. (Optional) Delete the original, untrimmed mesh from the DB Tree to keep your workspace clean.

### Step 4: Center the Map at (0, 0, 0)

To ensure the map loads correctly in Gazebo without floating away from the origin, you must center it:

1. Select your perfectly trimmed mesh in the **DB Tree**.
2. Look at the **Properties** panel on the bottom-left and scroll down to find the **Global Box Center** coordinates (`X`, `Y`, `Z`). Write these numbers down.
3. Go to **Edit > Apply Transformation** (or press `Ctrl + T`).
4. In the **Translation** section, enter the negative values of your center coordinates to pull the map back to zero
5. Click **OK**.

### Step 5: Export the Mesh

1. Select your centered mesh in the **DB Tree**.
2. Go to **File > Save**.
3. Choose your desired output format from the dropdown menu:
* **PLY (*.ply):** Retains full point cloud colors (Best for Sketchfab).
* **OBJ (*.obj):** Standard visual format (Best for Gazebo).
* **STL (*.stl):** Lightweight physics/collision format.
> **Important:** Use the file browser to navigate to a user-writable directory (e.g., `/home/username/Downloads/`). If you try to save directly in the default Snap folder, it will fail due to permission restrictions.
4. Click **Save**.
