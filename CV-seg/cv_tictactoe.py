import time
import cv2
import numpy as np
from collections import deque
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import log_say


class TicTacToeVision:
    """Computer vision system for detecting tic-tac-toe board state"""
    
    def __init__(self, camera_id=0):
        """
        Initialize the vision system.
        
        Args:
            camera_id (int): Camera device ID (0 for default webcam)
        """
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open camera {camera_id}")
        
        # Board corners for perspective transform (set during calibration)
        self.board_corners = None
        self.grid_size = 300  # Size of the warped grid
        
        # Previous board state for detecting new moves
        self.previous_state = [0] * 9
        
    def calibrate_board(self):
        """
        Calibrate by selecting the 4 corners of the tic-tac-toe board.
        Click corners in order: top-left, top-right, bottom-right, bottom-left
        """
        print("\n" + "=" * 60)
        print("BOARD CALIBRATION")
        print("=" * 60)
        print("Click on the 4 corners of your tic-tac-toe board in this order:")
        print("1. Top-left corner")
        print("2. Top-right corner")
        print("3. Bottom-right corner")
        print("4. Bottom-left corner")
        print("Press 'r' to reset, 'c' to confirm, 'q' to quit")
        print("=" * 60 + "\n")
        
        corners = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
                corners.append([x, y])
                print(f"Corner {len(corners)}/4 selected at ({x}, {y})")
        
        cv2.namedWindow('Calibration')
        cv2.setMouseCallback('Calibration', mouse_callback)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            display = frame.copy()
            
            # Draw selected corners
            for i, corner in enumerate(corners):
                cv2.circle(display, tuple(corner), 5, (0, 255, 0), -1)
                cv2.putText(display, str(i+1), (corner[0]+10, corner[1]+10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw lines between corners
            if len(corners) > 1:
                for i in range(len(corners)):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i+1) % len(corners)])
                    cv2.line(display, pt1, pt2, (0, 255, 0), 2)
            
            cv2.imshow('Calibration', display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):  # Reset
                corners = []
                print("Reset corners")
            elif key == ord('c') and len(corners) == 4:  # Confirm
                self.board_corners = np.array(corners, dtype=np.float32)
                print("✅ Calibration complete!")
                break
            elif key == ord('q'):  # Quit
                print("Calibration cancelled")
                cv2.destroyAllWindows()
                return False
        
        cv2.destroyAllWindows()
        return True
    
    def get_warped_board(self, frame):
        """Apply perspective transform to get top-down view of board"""
        if self.board_corners is None:
            return None
        
        dst_corners = np.array([
            [0, 0],
            [self.grid_size, 0],
            [self.grid_size, self.grid_size],
            [0, self.grid_size]
        ], dtype=np.float32)
        
        matrix = cv2.getPerspectiveTransform(self.board_corners, dst_corners)
        warped = cv2.warpPerspective(frame, matrix, (self.grid_size, self.grid_size))
        
        return warped
    
    def extract_cell(self, warped, row, col):
        """Extract a single cell from the warped board"""
        cell_size = self.grid_size // 3
        y1 = row * cell_size
        y2 = (row + 1) * cell_size
        x1 = col * cell_size
        x2 = (col + 1) * cell_size
        
        return warped[y1:y2, x1:x2]
    
    def detect_x_in_cell(self, cell):
        """
        Detect if cell contains an X mark.
        Uses edge detection and line detection to find X pattern.
        """
        # Convert to grayscale
        gray = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30,
                                minLineLength=20, maxLineGap=10)
        
        if lines is None or len(lines) < 2:
            return False
        
        # Check for diagonal lines (X pattern)
        diagonal_count = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
            
            # Check if line is roughly diagonal (30-60 degrees or 120-150 degrees)
            if (30 < angle < 60) or (120 < angle < 150):
                diagonal_count += 1
        
        # X should have at least 2 diagonal lines
        return diagonal_count >= 2
    
    def detect_board_state(self):
        """
        Detect the current state of all 9 cells.
        Returns: list of 9 integers (0=empty, 1=X, 2=O)
        """
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        warped = self.get_warped_board(frame)
        if warped is None:
            return None
        
        state = [0] * 9
        
        for row in range(3):
            for col in range(3):
                cell_idx = row * 3 + col
                cell = self.extract_cell(warped, row, col)
                
                # Check if cell contains X
                if self.detect_x_in_cell(cell):
                    state[cell_idx] = 1  # X detected
                # Note: We don't detect O since robot places those
        
        return state
    
    def wait_for_player_move(self, current_board, timeout=60):
        """
        Wait for player to make a move and detect it.
        
        Args:
            current_board: Current board state [0-9]
            timeout: Maximum seconds to wait
            
        Returns:
            int: Position (0-8) where move was detected, or None if timeout
        """
        print("\n👁️  Waiting for your move...")
        print("Place your X on the board and hold it steady for detection")
        
        start_time = time.time()
        stable_count = 0
        last_detected_move = None
        required_stable_frames = 10  # Require 10 consecutive detections
        
        while (time.time() - start_time) < timeout:
            state = self.detect_board_state()
            
            if state is None:
                continue
            
            # Find new X marks (compared to current board)
            new_moves = []
            for i in range(9):
                if state[i] == 1 and current_board[i] == 0:
                    new_moves.append(i)
            
            # Check if we detected exactly one new move
            if len(new_moves) == 1:
                detected_move = new_moves[0]
                
                if detected_move == last_detected_move:
                    stable_count += 1
                    if stable_count >= required_stable_frames:
                        print(f"✅ Move detected at position {detected_move + 1}!")
                        return detected_move
                else:
                    stable_count = 1
                    last_detected_move = detected_move
            else:
                stable_count = 0
                last_detected_move = None
            
            # Show live feed
            ret, frame = self.cap.read()
            if ret:
                display = frame.copy()
                if self.board_corners is not None:
                    # Draw board overlay
                    cv2.polylines(display, [self.board_corners.astype(np.int32)],
                                True, (0, 255, 0), 2)
                
                cv2.putText(display, "Waiting for move...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Game View', display)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        print("⏱️  Timeout waiting for move")
        return None
    
    def show_board_view(self, board_state):
        """Show the current board with overlay"""
        ret, frame = self.cap.read()
        if not ret:
            return
        
        display = frame.copy()
        
        if self.board_corners is not None:
            # Draw board boundary
            cv2.polylines(display, [self.board_corners.astype(np.int32)],
                        True, (0, 255, 0), 2)
            
            # Draw grid lines
            warped = self.get_warped_board(frame)
            if warped is not None:
                cell_size = self.grid_size // 3
                
                # Transform grid lines back to original perspective
                for i in range(1, 3):
                    # Horizontal lines
                    pt1 = np.array([[0, i * cell_size]], dtype=np.float32)
                    pt2 = np.array([[self.grid_size, i * cell_size]], dtype=np.float32)
                    
                    # Vertical lines
                    pt3 = np.array([[i * cell_size, 0]], dtype=np.float32)
                    pt4 = np.array([[i * cell_size, self.grid_size]], dtype=np.float32)
        
        cv2.imshow('Game View', display)
        cv2.waitKey(1)
    
    def release(self):
        """Release camera resources"""
        self.cap.release()
        cv2.destroyAllWindows()


def replay_episode(base_path, record_id, episode_idx, port="/dev/ttyACM1"):
    """
    Replay a robot episode from a local dataset.
    
    Args:
        base_path (str): Base path to lerobot datasets directory
        record_id (int): Record number (1-9)
        episode_idx (int): Episode ID (0-3)
        port (str): Serial port for robot connection
    
    Returns:
        bool: True if replay completed successfully, False otherwise
    """
    # Validate inputs
    if record_id == 'party':
        dataset_path = Path(base_path) / "recordparty"
    else:
        if not isinstance(record_id, int) or record_id < 1 or record_id > 9:
            raise ValueError(f"record_id must be an integer between 1 and 9, got {record_id}")
        dataset_path = Path(base_path) / f"record{record_id}"
    
    if not isinstance(episode_idx, int) or episode_idx < 0 or episode_idx > 3:
        raise ValueError(f"episode_idx must be an integer between 0 and 3, got {episode_idx}")
    
    if not dataset_path.exists():
        raise ValueError(f"Dataset path does not exist: {dataset_path}")
    
    print(f"\n📁 Loading dataset from: {dataset_path}")
    print(f"🎬 Record ID: {record_id}, Episode ID: {episode_idx}")
    
    robot = None
    try:
        # Initialize robot
        robot_config = SO101FollowerConfig(port=port)
        robot = SO101Follower(robot_config)
        robot.connect()
        print("✅ Robot connected successfully")
        
        # Load dataset
        dataset = LeRobotDataset(
            repo_id=str(dataset_path),
            episodes=[episode_idx]
        )
        
        episode_data = dataset.hf_dataset.filter(lambda x: x['episode_index'] == episode_idx)
        actions = episode_data.select_columns("action")
        num_frames = len(episode_data)
        
        print(f"📊 Total frames: {num_frames}, FPS: {dataset.fps}")
        print(f"🤖 Starting replay...\n")
        
        for idx in range(num_frames):
            t0 = time.perf_counter()
            
            action = {
                name: float(actions[idx]["action"][i]) 
                for i, name in enumerate(dataset.features["action"]["names"])
            }
            robot.send_action(action)
            
            if (idx + 1) % 10 == 0:
                print(f"Frame {idx + 1}/{num_frames}")
            
            busy_wait(1.0 / dataset.fps - (time.perf_counter() - t0))
        
        print(f"\n✅ Replay completed!")
        return True
        
    except Exception as e:
        print(f"❌ Error during replay: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        if robot is not None:
            try:
                robot.disconnect()
                print("🔌 Robot disconnected")
            except:
                pass


class RobotTicTacToe:
    def __init__(self, base_path, port="/dev/ttyACM1", camera_id=0, use_cv=True):
        """
        Initialize the Robot Tic Tac Toe game.
        
        Args:
            base_path (str): Base path to lerobot datasets directory
            port (str): Serial port for robot connection
            camera_id (int): Camera device ID
            use_cv (bool): Whether to use computer vision for move detection
        """
        self.board = [0] * 9
        self.human = 1
        self.computer = 2
        self.episode_queue = deque([0, 1, 2, 3])
        self.base_path = base_path
        self.port = port
        self.computer_moves = []
        self.use_cv = use_cv
        
        if not Path(base_path).exists():
            raise ValueError(f"Base path does not exist: {base_path}")
        
        # Initialize vision system
        if use_cv:
            try:
                self.vision = TicTacToeVision(camera_id)
                if not self.vision.calibrate_board():
                    print("⚠️  Calibration failed, falling back to manual input")
                    self.use_cv = False
                    self.vision.release()
            except Exception as e:
                print(f"⚠️  Could not initialize vision: {e}")
                print("Falling back to manual input")
                self.use_cv = False
        
    def print_board(self):
        """Display the current game board"""
        print("\n")
        for i in range(3):
            row = []
            for j in range(3):
                cell_idx = i * 3 + j
                cell = self.board[cell_idx]
                
                if cell == 0:
                    row.append(" ")
                elif cell == 1:
                    row.append("X")
                else:
                    row.append(str(cell_idx + 1))
            
            print(f" {row[0]} | {row[1]} | {row[2]} ")
            if i < 2:
                print("-----------")
        print("\n")
    
    def is_winner(self, player):
        """Check if the specified player has won"""
        winning_combos = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        
        for combo in winning_combos:
            if all(self.board[i] == player for i in combo):
                return True
        return False
    
    def is_board_full(self):
        """Check if the board is full"""
        return all(cell != 0 for cell in self.board)
    
    def get_available_moves(self):
        """Return list of available move positions"""
        return [i for i in range(9) if self.board[i] == 0]
    
    def evaluate(self):
        """Evaluate the board state for minimax"""
        if self.is_winner(self.computer):
            return 10
        elif self.is_winner(self.human):
            return -10
        else:
            return 0
    
    def minimax(self, depth, is_maximizing):
        """Minimax algorithm to find the best move"""
        score = self.evaluate()
        
        if score == 10:
            return score - depth
        if score == -10:
            return score + depth
        if self.is_board_full():
            return 0
        
        if is_maximizing:
            best_score = -float('inf')
            for move in self.get_available_moves():
                self.board[move] = self.computer
                score = self.minimax(depth + 1, False)
                self.board[move] = 0
                best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for move in self.get_available_moves():
                self.board[move] = self.human
                score = self.minimax(depth + 1, True)
                self.board[move] = 0
                best_score = min(score, best_score)
            return best_score
    
    def get_best_move(self):
        """Find the best move for the computer using minimax"""
        best_score = -float('inf')
        best_move = None
        
        for move in self.get_available_moves():
            self.board[move] = self.computer
            score = self.minimax(0, False)
            self.board[move] = 0
            
            if score > best_score:
                best_score = score
                best_move = move
        
        return best_move
    
    def player_move(self):
        """Get player's move using CV or manual input"""
        if self.use_cv:
            # Use computer vision
            move = self.vision.wait_for_player_move(self.board, timeout=60)
            
            if move is None:
                print("⚠️  No move detected, switching to manual input")
                return self.manual_player_move()
            
            self.board[move] = self.human
            print(f"✅ You placed X at position {move + 1}")
            return True
        else:
            return self.manual_player_move()
    
    def manual_player_move(self):
        """Get player's move via keyboard input"""
        while True:
            try:
                move = int(input("Enter your move (1-9): ")) - 1
                if 0 <= move <= 8 and self.board[move] == 0:
                    self.board[move] = self.human
                    print(f"You placed X at position {move + 1}")
                    return True
                else:
                    print("That position is already taken or invalid. Try again!")
            except ValueError:
                print("Please enter a number between 1 and 9.")
    
    def computer_move(self):
        """Make the computer's move and replay corresponding episode"""
        available = self.get_available_moves()
        
        if not available:
            return False
        
        if not self.episode_queue:
            print("❌ No more episodes available!")
            return False
        
        episode_idx = self.episode_queue.popleft()
        move = self.get_best_move()
        self.board[move] = self.computer
        self.computer_moves.append((move, episode_idx))
        
        record_number = move + 1
        
        print(f"\n🤖 Computer placed O at position {move + 1}")
        print(f"📊 Using Episode ID: {episode_idx}, Record: {record_number}")
        print(f"📋 Remaining episodes: {list(self.episode_queue)}")
        print("\n🔄 Replaying robot movement...\n")
        
        success = replay_episode(self.base_path, record_number, episode_idx, self.port)
        
        if not success:
            print("⚠️  Robot replay failed, but game continues...\n")
        
        return True
    
    def play(self):
        """Main game loop"""
        print("=" * 60)
        print("Welcome to Robot Tic Tac Toe with Computer Vision!")
        print("=" * 60)
        print("\nYou are X, Computer is O")
        if self.use_cv:
            print("👁️  Computer vision is ENABLED - place your X on the board")
        else:
            print("⌨️  Manual input mode")
        print("\nPositions are numbered 1-9:")
        print(" 1 | 2 | 3 ")
        print("-----------")
        print(" 4 | 5 | 6 ")
        print("-----------")
        print(" 7 | 8 | 9 ")
        
        while True:
            self.print_board()
            
            # Show camera view if using CV
            if self.use_cv:
                self.vision.show_board_view(self.board)
            
            # Player's turn
            self.player_move()
            self.print_board()
            
            if self.is_winner(self.human):
                print("🎉 You won! Congratulations!")
                break
            
            if self.is_board_full():
                print("It's a draw!")
                break
            
            # Computer's turn
            print("Computer is thinking...")
            if not self.computer_move():
                print("Game ended: No more episodes available")
                break
            
            self.print_board()
            
            if self.is_winner(self.computer):
                print("💻 Computer won! Better luck next time!")
                replay_episode(self.base_path, 'party', 0, self.port)
                break
            
            if self.is_board_full():
                print("It's a draw!")
                break
        
        self.print_summary()
        
        # Cleanup
        if self.use_cv:
            self.vision.release()
    
    def print_summary(self):
        """Print a summary of computer moves"""
        print("\n" + "=" * 60)
        print("Game Summary - Computer Moves")
        print("=" * 60)
        for move_num, (position, episode_id) in enumerate(self.computer_moves, 1):
            print(f"Move {move_num}: Position {position + 1} (Record {position + 1}), Episode ID: {episode_id}")
        print("=" * 60 + "\n")


def main():
    print("=" * 60)
    print("Robot Tic Tac Toe with Computer Vision")
    print("=" * 60)
    
    base_path = input("Enter base dataset path (default: /home/taksh/lerobot_datasets): ").strip()
    if not base_path:
        base_path = "/home/taksh/lerobot_datasets"
    
    if not Path(base_path).exists():
        print(f"❌ Error: Base path does not exist: {base_path}")
        return
    
    port = input("Enter robot serial port (default: /dev/ttyACM1): ").strip()
    if not port:
        port = "/dev/ttyACM1"
    
    use_cv_input = input("Use computer vision for move detection? (yes/no, default: yes): ").strip().lower()
    use_cv = use_cv_input != "no" and use_cv_input != "n"
    
    camera_id = 0
    if use_cv:
        camera_input = input("Enter camera ID (default: 0): ").strip()
        if camera_input:
            try:
                camera_id = int(camera_input)
            except ValueError:
                print("Invalid camera ID, using default (0)")
    
    while True:
        try:
            game = RobotTicTacToe(base_path, port, camera_id, use_cv)
            game.play()
        except ValueError as e:
            print(f"❌ Error: {e}")
            break
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            break
        
        play_again = input("\nDo you want to play again? (yes/no): ").lower().strip()
        if play_again != "yes" and play_again != "y":
            print("Thanks for playing!")
            break


if __name__ == "__main__":
    main()