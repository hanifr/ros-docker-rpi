class ROS3DViewer {
    constructor(containerId, rosUrl, tf2Url) {
        this.containerId = containerId;
        this.rosUrl = rosUrl;
        this.tf2Url = tf2Url;
        this.ros = null;
        this.viewer = null;
        this.tfClient = null;
        this.urdfClient = null;
        this.gridClient = null;
        this.laserClient = null;
        this.pathClient = null;
        this.markerArrayClient = null;
        this.interactiveMarkerClient = null;
        
        this.isConnected = false;
        this.robotLoaded = false;
        
        this.init();
    }

    init() {
        console.log('🚀 Initializing ROS3D Viewer...');
        this.connectToROS();
    }

    connectToROS() {
        // Connect to main ROS Bridge
        this.ros = new ROSLIB.Ros({
            url: this.rosUrl
        });

        this.ros.on('connection', () => {
            console.log('✅ Connected to ROS Bridge');
            this.isConnected = true;
            this.setupViewer();
            this.updateConnectionStatus('connected');
        });

        this.ros.on('error', (error) => {
            console.error('❌ ROS Bridge Error:', error);
            this.isConnected = false;
            this.updateConnectionStatus('error', error.toString());
        });

        this.ros.on('close', () => {
            console.log('🔌 ROS Bridge Disconnected');
            this.isConnected = false;
            this.updateConnectionStatus('disconnected');
        });
    }

    setupViewer() {
        console.log('🎯 Setting up 3D viewer...');
        
        // Create main 3D viewer
        this.viewer = new ROS3D.Viewer({
            divID: this.containerId,
            width: 800,
            height: 600,
            antialias: true,
            background: 0x111111,
            alpha: 1,
            cameraPose: {
                x: 3,
                y: 3,
                z: 3
            },
            cameraZoomSpeed: 0.5
        });

        // Add coordinate frame grid
        this.addGrid();
        
        // Setup TF client for coordinate transforms
        this.setupTFClient();
        
        // Load robot URDF model
        this.loadRobotModel();
        
        // Setup sensor visualizations
        this.setupSensorVisualizations();
        
        // Setup interactive markers
        this.setupInteractiveMarkers();
        
        // Add lighting
        this.setupLighting();
        
        // Setup camera controls
        this.setupCameraControls();
        
        console.log('✅ 3D viewer setup complete');
    }

    addGrid() {
        // Add a coordinate grid
        this.viewer.addObject(new ROS3D.Grid({
            color: 0x444444,
            cellSize: 1,
            num_cells: 20
        }));

        // Add coordinate axes
        this.viewer.addObject(new ROS3D.Axes({
            shaftRadius: 0.02,
            headRadius: 0.05,
            headLength: 0.1,
            scale: 1.2
        }));
    }

    setupTFClient() {
        console.log('🔗 Setting up TF client...');
        
        this.tfClient = new ROSLIB.TFClient({
            ros: this.ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: '/odom'  // Use odom as fixed frame for navigation
        });

        // Log TF updates
        this.tfClient.on('change', () => {
            // console.log('🔄 TF update received');
        });
    }

    loadRobotModel() {
        console.log('🤖 Loading robot URDF model...');
        
        // Create URDF client to load and display robot model
        this.urdfClient = new ROS3D.UrdfClient({
            ros: this.ros,
            tfClient: this.tfClient,
            path: 'http://resources.robotwebtools.org/',
            rootObject: this.viewer.scene,
            loader: ROS3D.COLLADA_LOADER_2,
            // Try to load robot description from parameter server
            param: 'robot_description'
        });

        // Handle URDF loading events
        this.urdfClient.on('change', () => {
            console.log('✅ Robot URDF model loaded successfully');
            this.robotLoaded = true;
            this.updateRobotStatus('loaded');
        });

        // If robot_description parameter doesn't exist, create a simple robot
        setTimeout(() => {
            if (!this.robotLoaded) {
                console.log('⚠️ No URDF found, creating simple robot visualization');
                this.createSimpleRobot();
            }
        }, 3000);
    }

    createSimpleRobot() {
        // Create a simple robot representation if URDF is not available
        const robotGroup = new THREE.Group();
        
        // Robot base
        const baseGeometry = new THREE.BoxGeometry(0.6, 0.4, 0.2);
        const baseMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x007acc,
            transparent: true,
            opacity: 0.8
        });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = 0.1;
        robotGroup.add(base);

        // Direction indicator (arrow)
        const arrowGeometry = new THREE.ConeGeometry(0.08, 0.2, 8);
        const arrowMaterial = new THREE.MeshLambertMaterial({ color: 0xff3333 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(0.35, 0.2, 0);
        arrow.rotation.z = -Math.PI / 2;
        robotGroup.add(arrow);

        // Add to scene with TF frame
        const robotTF = new ROS3D.SceneNode({
            tfClient: this.tfClient,
            frameID: '/base_link',
            object: robotGroup
        });

        this.viewer.scene.add(robotTF);
        this.robotLoaded = true;
        console.log('✅ Simple robot visualization created');
    }

    setupSensorVisualizations() {
        console.log('📡 Setting up sensor visualizations...');

        // Laser scan visualization
        try {
            this.laserClient = new ROS3D.LaserScan({
                ros: this.ros,
                topic: '/scan',
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                pointRatio: 2,
                pointSize: 0.02,
                color: 0xff0000,
                max_pts: 1000
            });
            console.log('✅ Laser scan visualization setup');
        } catch (error) {
            console.log('⚠️ Laser scan not available');
        }

        // Point cloud visualization
        try {
            const pointCloudClient = new ROS3D.PointCloud2({
                ros: this.ros,
                topic: '/camera/depth/points',
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                pointRatio: 1,
                pointSize: 0.01,
                max_pts: 5000
            });
            console.log('✅ Point cloud visualization setup');
        } catch (error) {
            console.log('⚠️ Point cloud not available');
        }

        // Path/trajectory visualization
        try {
            this.pathClient = new ROS3D.Path({
                ros: this.ros,
                topic: '/move_base/GlobalPlanner/plan',
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                color: 0x00ff00
            });
            console.log('✅ Path visualization setup');
        } catch (error) {
            console.log('⚠️ Path visualization not available');
        }

        // Occupancy grid (map) visualization
        try {
            this.gridClient = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.viewer.scene,
                topic: '/map',
                tfClient: this.tfClient,
                opacity: 0.7,
                color: {
                    r: 255,
                    g: 255,
                    b: 255
                }
            });
            console.log('✅ Occupancy grid visualization setup');
        } catch (error) {
            console.log('⚠️ Map visualization not available');
        }
    }

    setupInteractiveMarkers() {
        console.log('🎯 Setting up interactive markers...');
        
        try {
            this.interactiveMarkerClient = new ROS3D.InteractiveMarkerClient({
                ros: this.ros,
                tfClient: this.tfClient,
                topic: '/basic_controls',
                camera: this.viewer.camera,
                rootObject: this.viewer.selectableObjs
            });
            console.log('✅ Interactive markers setup');
        } catch (error) {
            console.log('⚠️ Interactive markers not available');
        }

        // Marker array for custom markers
        try {
            this.markerArrayClient = new ROS3D.MarkerArrayClient({
                ros: this.ros,
                tfClient: this.tfClient,
                topic: '/visualization_marker_array',
                rootObject: this.viewer.scene,
                path: 'http://resources.robotwebtools.org/'
            });
            console.log('✅ Marker array visualization setup');
        } catch (error) {
            console.log('⚠️ Marker array not available');
        }
    }

    setupLighting() {
        // Add better lighting to the scene
        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
        this.viewer.scene.add(ambientLight);

        const directionalLight1 = new THREE.DirectionalLight(0xffffff, 0.6);
        directionalLight1.position.set(10, 10, 5);
        directionalLight1.shadow.mapSize.width = 1024;
        directionalLight1.shadow.mapSize.height = 1024;
        this.viewer.scene.add(directionalLight1);

        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.3);
        directionalLight2.position.set(-10, -10, -5);
        this.viewer.scene.add(directionalLight2);
    }

    setupCameraControls() {
        // Add keyboard controls for camera
        document.addEventListener('keydown', (event) => {
            if (event.target.tagName.toLowerCase() === 'input') return;
            
            const camera = this.viewer.camera;
            const speed = 0.5;
            
            switch(event.key.toLowerCase()) {
                case 'q': // Camera up
                    camera.position.y += speed;
                    break;
                case 'e': // Camera down
                    camera.position.y -= speed;
                    break;
                case 'r': // Reset camera
                    camera.position.set(3, 3, 3);
                    camera.lookAt(0, 0, 0);
                    break;
            }
        });
    }

    // Method to add custom markers programmatically
    addCustomMarker(position, color = 0xff0000, type = 'sphere') {
        let geometry;
        
        switch(type) {
            case 'sphere':
                geometry = new THREE.SphereGeometry(0.1, 8, 6);
                break;
            case 'cube':
                geometry = new THREE.BoxGeometry(0.2, 0.2, 0.2);
                break;
            case 'arrow':
                geometry = new THREE.ConeGeometry(0.05, 0.2, 8);
                break;
            default:
                geometry = new THREE.SphereGeometry(0.1, 8, 6);
        }
        
        const material = new THREE.MeshLambertMaterial({ color: color });
        const marker = new THREE.Mesh(geometry, material);
        marker.position.set(position.x, position.y, position.z);
        
        this.viewer.scene.add(marker);
        
        // Remove marker after 5 seconds
        setTimeout(() => {
            this.viewer.scene.remove(marker);
        }, 5000);
    }

    // Method to update robot pose manually
    updateRobotPose(pose) {
        if (this.robotLoaded) {
            // This would typically be handled by TF, but can be used for debugging
            console.log('Robot pose update:', pose);
        }
    }

    // Status update methods
    updateConnectionStatus(status, message = '') {
        const statusElement = document.getElementById('connection-status');
        if (statusElement) {
            let statusText = '';
            let className = 'status ';
            
            switch(status) {
                case 'connected':
                    statusText = '✅ Connected to ROS Bridge';
                    className += 'connected';
                    break;
                case 'disconnected':
                    statusText = '🔌 Disconnected from ROS Bridge';
                    className += 'disconnected';
                    break;
                case 'error':
                    statusText = '❌ Connection Error: ' + message;
                    className += 'disconnected';
                    break;
            }
            
            statusElement.textContent = statusText;
            statusElement.className = className;
        }
    }

    updateRobotStatus(status) {
        const robotStatusElement = document.getElementById('robot-status');
        if (robotStatusElement) {
            let statusText = '';
            
            switch(status) {
                case 'loaded':
                    statusText = '🤖 Robot model loaded successfully';
                    break;
                case 'loading':
                    statusText = '⏳ Loading robot model...';
                    break;
                case 'error':
                    statusText = '❌ Failed to load robot model';
                    break;
            }
            
            robotStatusElement.textContent = statusText;
        }
    }

    // Cleanup method
    destroy() {
        if (this.ros) {
            this.ros.close();
        }
    }
}

// Export for use in other scripts
window.ROS3DViewer = ROS3DViewer;