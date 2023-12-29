/*
 * flood.c
 *
 *  Created on: Mar 20, 2018
 *      Author: jlammwe
 */

#include "flood.h"

extern int16_t debug[100];
extern uint8_t ReadButton;

extern uint8_t TurnFlag;
extern uint8_t FinishFlag;

uint32_t BeforeTurnTicks = 400;
uint32_t AfterTurnTicks = 100;

uint8_t DistanceMovement = 70;

int32_t PreviousLeftLaserValue = 0;
int32_t PreviousRightLaserValue = 0;
int32_t LeftLaserValue = 0;
int32_t RightLaserValue = 0;

uint8_t DebugMode = 0;
extern volatile uint8_t RemarkAfterTurnMode;
extern volatile uint8_t RemarkWallMode;
extern volatile uint8_t RemarkAfterBackwardMode;

void ShowTick(uint32_t ticks)
{
	uint8_t n = (uint8_t)(ticks / 100);

	for (uint8_t i = 0; i < n; i++)
	{
		AML_DebugDevice_TurnOnLED(i);
	}

	HAL_Delay(500);

	AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
}

void SetBeforeTurnTicks(uint32_t ticks)
{
	if (ticks == 0)
	{
		BeforeTurnTicks -= 100;
	}
	else
	{
		BeforeTurnTicks += 100;
	}

	ShowTick(BeforeTurnTicks);
}

void SetAfterTurnTicks(uint32_t ticks)
{
	if (ticks == 0)
	{
		AfterTurnTicks -= 100;
	}
	else
	{
		AfterTurnTicks += 100;
	}

	ShowTick(AfterTurnTicks);
}

void init_distance_maze(struct dist_maze *dm, struct coor *c, int center)
{
	// If we are trying to get to the center
	// Initialize the distance maze with the center four cells as zero
	if (center == 1)
	{
		for (int i = 0; i < 16; i++)
		{
			for (int j = 0; j < 16; j++)
			{
				if (i <= 7 && j <= 7)
					dm->distance[i][j] = ((7 - i) + (7 - j));
				if (i <= 7 && j > 7)
					dm->distance[i][j] = ((7 - i) + (j - 8));
				if (i > 7 && j <= 7)
					dm->distance[i][j] = ((i - 8) + (7 - j));
				if (i > 7 && j > 7)
					dm->distance[i][j] = ((i - 8) + (j - 8));
			}
		}
	}
	// Generalized floodfill for any other cell
	else
	{
		for (int i = 0; i < 16; i++)
		{
			for (int j = 0; j < 16; j++)
			{
				// get the distance to target cell
				int x = i - c->x;
				if (x < 0)
					x = -x;
				int y = j - c->y;
				if (y < 0)
					y = -y;
				dm->distance[i][j] = x + y;
			}
		}
	}
}

// Initialize all cells to unvisited
void init_wall_maze(struct wall_maze *wm)
{
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < 16; j++)
		{
			wm->cells[i][j].walls[NORTH] = 0;
			wm->cells[i][j].walls[EAST] = 0;
			wm->cells[i][j].walls[SOUTH] = 0;
			wm->cells[i][j].walls[WEST] = 0;
			wm->cells[i][j].visited = 0;
			// put the walls surrounding the maze
			if (i == 0)
				wm->cells[i][j].walls[WEST] = 1;
			if (j == 0)
				wm->cells[i][j].walls[SOUTH] = 1;
			if (i == 15)
				wm->cells[i][j].walls[EAST] = 1;
			if (j == 15)
				wm->cells[i][j].walls[NORTH] = 1;
		}
	}
}

// Constructor for initializing coordinates
void init_coor(struct coor *c, int x, int y)
{
	c->x = x;
	c->y = y;
}

// Pop top of stack
struct coor pop_stack(struct stack *s)
{
	s->index = s->index - 1;
	return s->array[s->index + 1];
}

// Push to top of stack
void push_stack(struct stack *s, struct coor c)
{
	s->index = s->index + 1;
	s->array[s->index] = c;
}

void uncontrolledAdvanceTicks(uint32_t ticks)
{
	uint32_t InitTime = HAL_GetTick();
	uint32_t PreviousTime = InitTime;
	uint32_t delay;

	uint8_t BreakFlag = 0;

	if (ticks > 1000)
	{
		delay = 2000;
	}
	else
	{
		delay = 800;
		AML_MotorControl_SetMouseSpeed(9);
	}

	// uint32_t encoder_val = 0;
	AML_Encoder_ResetLeftValue();

	HAL_Delay(35);

	if (ticks > 1000)
	{
		AML_MotorControl_TurnOnWallFollow();

		PreviousLeftLaserValue = AML_LaserSensor_ReadSingleWithFillter(BL);
		PreviousRightLaserValue = AML_LaserSensor_ReadSingleWithFillter(BR);

		while (((uint32_t)AML_Encoder_GetLeftValue()) < ticks && (HAL_GetTick() - InitTime) < delay && AML_LaserSensor_ReadSingleWithFillter(FF) > 35)
		{
			if ((HAL_GetTick() - PreviousTime) > 45)
			{
				LeftLaserValue = AML_LaserSensor_ReadSingleWithFillter(BL);
				RightLaserValue = AML_LaserSensor_ReadSingleWithFillter(BR);

				if (((RemarkWallMode == 1) && ((LeftLaserValue > WALL_NOT_IN_LEFT && PreviousLeftLaserValue < WALL_IN_LEFT) || (RightLaserValue > WALL_NOT_IN_RIGHT && PreviousRightLaserValue < WALL_IN_RIGHT))) ||
					((RemarkWallMode == 1) && ((LeftLaserValue < WALL_IN_LEFT && PreviousLeftLaserValue > WALL_NOT_IN_LEFT && AML_LaserSensor_ReadSingleWithFillter(FL) < WALL_IN_FRONT_LEFT) || (RightLaserValue < WALL_IN_RIGHT && PreviousRightLaserValue > WALL_NOT_IN_RIGHT && AML_LaserSensor_ReadSingleWithFillter(FR) < WALL_IN_FRONT_RIGHT))))

				{
					AML_Encoder_ResetLeftValue();
					BreakFlag = 1;
					break;
				}
				else
				{
					PreviousLeftLaserValue = LeftLaserValue;
					PreviousRightLaserValue = RightLaserValue;
					PreviousTime = HAL_GetTick();
				}
			}
		}

		if (BreakFlag)
		{
			// AML_MotorControl_TurnOffWallFollow();
			// AML_MotorControl_ShortBreak('F');

			AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

			// AML_MotorControl_MoveForward_mm((uint16_t)DistanceMovement);

			if ((LeftLaserValue < WALL_IN_LEFT && PreviousLeftLaserValue > WALL_NOT_IN_LEFT) || (RightLaserValue < WALL_IN_RIGHT && PreviousRightLaserValue > WALL_NOT_IN_RIGHT))
			{
				AML_MotorControl_MoveForward_mm(75);
			}
			else if ((LeftLaserValue > WALL_NOT_IN_LEFT && PreviousLeftLaserValue < WALL_IN_LEFT) || (RightLaserValue > WALL_NOT_IN_RIGHT && PreviousRightLaserValue < WALL_IN_RIGHT))
			{
				AML_MotorControl_MoveForward_mm(45);
			}

			AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
			BreakFlag = 0;
		}
	}
	else
	{
		AML_MotorControl_TurnOnWallFollow();

		while (((uint32_t)AML_Encoder_GetLeftValue()) < ticks && HAL_GetTick() - InitTime < delay)
			;
	}

	AML_MotorControl_TurnOffWallFollow();

	// if (ticks > 1000)
	// {
	// 	AML_MotorControl_ShortBreak('F');
	// }

	AML_MotorControl_SetMouseSpeed(16);
	AML_Encoder_ResetLeftValue();
}

// move one cell forward and also poll adc for wall values
void advanceTicksFlood(uint32_t ticks, int d, struct coor *c, struct wall_maze *wm)
{
	// uint32_t encoder_val = MAX_ENCODER_VALUE;

	// uint32_t encoder_val = 0;

	// resetLeftEncoder();
	AML_Encoder_ResetLeftValue();

	// put walls up for the cell we are moving to. For each direction put a wall to the
	// east and the west.
	switch (d)
	{
	case NORTH:
		wm->cells[c->x][c->y].walls[WEST] = 1;
		wm->cells[c->x][c->y].walls[EAST] = 1;
		// checks for out of bounds
		if (c->x > 0)
			wm->cells[c->x - 1][c->y].walls[EAST] = 1;
		if (c->x < 15)
			wm->cells[c->x + 1][c->y].walls[WEST] = 1;
		break;
	case EAST:
		wm->cells[c->x][c->y].walls[NORTH] = 1;
		wm->cells[c->x][c->y].walls[SOUTH] = 1;
		if (c->y > 0)
			wm->cells[c->x][c->y - 1].walls[NORTH] = 1;
		if (c->y < 15)
			wm->cells[c->x][c->y + 1].walls[SOUTH] = 1;
		break;
	case SOUTH:
		wm->cells[c->x][c->y].walls[WEST] = 1;
		wm->cells[c->x][c->y].walls[EAST] = 1;
		if (c->x > 0)
			wm->cells[c->x - 1][c->y].walls[EAST] = 1;
		if (c->x < 15)
			wm->cells[c->x + 1][c->y].walls[WEST] = 1;
		break;
	case WEST:
		wm->cells[c->x][c->y].walls[NORTH] = 1;
		wm->cells[c->x][c->y].walls[SOUTH] = 1;
		if (c->y > 0)
			wm->cells[c->x][c->y - 1].walls[NORTH] = 1;
		if (c->y < 15)
			wm->cells[c->x][c->y + 1].walls[SOUTH] = 1;
		break;
	default:
		break;
	}

	uint32_t InitTime = HAL_GetTick();
	uint32_t PreviousTime = InitTime;

	uint8_t BreakFlag = 0;

	HAL_Delay(20);

	PreviousLeftLaserValue = AML_LaserSensor_ReadSingleWithFillter(BL);
	PreviousRightLaserValue = AML_LaserSensor_ReadSingleWithFillter(BR);

	// while we have not finished moving one cell length
	// while (encoder_val > (MAX_ENCODER_VALUE - ticks))
	while (((uint32_t)AML_Encoder_GetLeftValue()) < ticks && HAL_GetTick() - InitTime < 2500 && AML_LaserSensor_ReadSingleWithFillter(FF) > 45)
	{
		if (HAL_GetTick() - PreviousTime > 45)
		{
			LeftLaserValue = AML_LaserSensor_ReadSingleWithFillter(BL);
			RightLaserValue = AML_LaserSensor_ReadSingleWithFillter(BR);

			if (((RemarkWallMode == 1) && ((LeftLaserValue > WALL_NOT_IN_LEFT && PreviousLeftLaserValue < WALL_IN_LEFT) || (RightLaserValue > WALL_NOT_IN_RIGHT && PreviousRightLaserValue < WALL_IN_RIGHT))) ||
				((RemarkWallMode == 1) && ((LeftLaserValue < WALL_IN_LEFT && PreviousLeftLaserValue > WALL_NOT_IN_LEFT && AML_LaserSensor_ReadSingleWithFillter(FL) < WALL_IN_FRONT_LEFT) || (RightLaserValue < WALL_IN_RIGHT && PreviousRightLaserValue > WALL_NOT_IN_RIGHT && AML_LaserSensor_ReadSingleWithFillter(FR) < WALL_IN_FRONT_RIGHT))))
			{
				AML_Encoder_ResetLeftValue();
				BreakFlag = 1;

				break;
			}
			else
			{
				PreviousLeftLaserValue = LeftLaserValue;
				PreviousRightLaserValue = RightLaserValue;
				PreviousTime = HAL_GetTick();
			}
		}

		// if we have moved half of a cell length
		// if (encoder_val < (MAX_ENCODER_VALUE - (ticks / 2)))
		if (AML_Encoder_GetLeftValue() > (int16_t)(ticks * 0.75f) && ((uint32_t)AML_Encoder_GetLeftValue()) < (int16_t)(ticks * 0.9f)) // can check
		{
			// check for walls to the east and the west
			switch (d)
			{
			case NORTH:
				checkForWalls(wm, c, d, NORTH, EAST, SOUTH, WEST);
				break;
			case EAST:
				checkForWalls(wm, c, d, EAST, SOUTH, WEST, NORTH);
				break;
			case SOUTH:
				checkForWalls(wm, c, d, SOUTH, WEST, NORTH, EAST);
				break;
			case WEST:
				checkForWalls(wm, c, d, WEST, NORTH, EAST, SOUTH);
				break;
			default:
				break;
			}
		}
		// setLeftEncoderValue(TIM2->CNT);
		// encoder_val = getLeftEncoderValue();
		// encoder_val = AML_Encoder_GetLeftValue();
	}

	if ((HAL_GetTick() - InitTime > 2500 || AML_LaserSensor_ReadSingleWithFillter(FF) < 40) && !BreakFlag)
	{
		switch (d)
		{
		case NORTH:
			checkForWalls(wm, c, d, NORTH, EAST, SOUTH, WEST);
			break;
		case EAST:
			checkForWalls(wm, c, d, EAST, SOUTH, WEST, NORTH);
			break;
		case SOUTH:
			checkForWalls(wm, c, d, SOUTH, WEST, NORTH, EAST);
			break;
		case WEST:
			checkForWalls(wm, c, d, WEST, NORTH, EAST, SOUTH);
			break;
		default:
			break;
		}

		// AML_MotorControl_TurnOffWallFollow();
		// AML_MotorControl_ShortBreak('F');
	}

	if (BreakFlag)
	{
		AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
		// AML_MotorControl_MoveForward_mm((uint16_t)DistanceMovement);

		if ((LeftLaserValue < WALL_IN_LEFT && PreviousLeftLaserValue > WALL_NOT_IN_LEFT) || (RightLaserValue < WALL_IN_RIGHT && PreviousRightLaserValue > WALL_NOT_IN_RIGHT))
		{
			AML_MotorControl_MoveForward_mm(70);
		}
		else if ((LeftLaserValue > WALL_NOT_IN_LEFT && PreviousLeftLaserValue < WALL_IN_LEFT) || (RightLaserValue > WALL_NOT_IN_RIGHT && PreviousRightLaserValue < WALL_IN_RIGHT))
		{
			AML_MotorControl_MoveForward_mm(45);
		}

		HAL_Delay(21);
		AML_LaserSensor_ReadAll();

		for (uint8_t i = 0; i < 2; i++)
		{
			switch (d)
			{
			case NORTH:
				checkForWalls(wm, c, d, NORTH, EAST, SOUTH, WEST);
				break;
			case EAST:
				checkForWalls(wm, c, d, EAST, SOUTH, WEST, NORTH);
				break;
			case SOUTH:
				checkForWalls(wm, c, d, SOUTH, WEST, NORTH, EAST);
				break;
			case WEST:
				checkForWalls(wm, c, d, WEST, NORTH, EAST, SOUTH);
				break;
			default:
				break;
			}
		}

		AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

		BreakFlag = 0;
	}

	AML_Encoder_ResetLeftValue();
}

/* Parameters
 * dm = the distance array containing the distances to the spot we want to
 * flood to
 * x, y = the current coordinates
 * wm = structure containing information on the cell walls and visitation
 *
 */
int floodFill(struct dist_maze *dm, struct coor *c, struct wall_maze *wm, int a, int direction, struct stack *upst)
{
	// Disable tracking interrupts because we do not want to move yet
	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	// coordinate for future use in popping stack
	int next_move;
	struct coor next;

	uint16_t TempTicks = 0;
	uint16_t ReCalibFlag = 0;

	// set the base speed of traversal
	// setBaseSpeed(40);
	// AML_MotorControl_SetMouseSpeed(40);

	// while we are not at target destination
	while (1)
	{
		// uint8_t WallFlag = wm->cells[c->x][c->y].walls[direction];

		// update coordinates for next cell we are going to visit
		switch (direction)
		{
		case NORTH:
			c->y += 1;
			break;
		case EAST:
			c->x += 1;
			break;
		case SOUTH:
			c->y -= 1;
			break;
		case WEST:
			c->x -= 1;
			break;
		default:
			break;
		}

		// If we haven't visited the next cell
		if (wm->cells[c->x][c->y].visited == 0)
		{
			// advance one cell
			advanceOneCell(direction, c, wm);

			// check for wall straight ahead
			// if (getLeftADCValue() >= FLOOD_WALL_IN_FRONT_LEFT &&
			// 	getRightADCValue() >= FLOOD_WALL_IN_FRONT_RIGHT)
			if (AML_LaserSensor_ReadSingleWithFillter(FF) < WALL_IN_FRONT)
			{
				// put wall ahead of us
				wm->cells[c->x][c->y].walls[direction] = 1;
				switch (direction)
				{
				// put wall ahead of us in the cell ahead of us
				case NORTH:
					if (c->y + 1 < 16)
						wm->cells[c->x][c->y + 1].walls[SOUTH] = 1;

					break;
				case EAST:
					if (c->x + 1 < 16)
						wm->cells[c->x + 1][c->y].walls[WEST] = 1;

					break;
				case SOUTH:
					if (c->y - 1 > -1)
						wm->cells[c->x][c->y - 1].walls[NORTH] = 1;

					break;
				case WEST:
					if (c->x - 1 > -1)
						wm->cells[c->x - 1][c->y].walls[EAST] = 1;

					break;
				}
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			}
			else
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

				// update the current cell as visited
				wm->cells[c->x][c->y].visited = 1;
		}
		else
		{
			// advance one cell without scanning for walls
			advanceOneCellVisited();
		}

		// if we are at target destination
		if (dm->distance[c->x][c->y] == 0)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			// AML_MotorControl_Stop();
			// AML_MotorControl_ShortBreak('F');

			return direction;
			break;
		}

		// check if there is a neighbor with one less distance
		// next_move is the direction we should move next
		next_move = minusOneNeighbor(dm, wm, c, upst, a);

		// If we couldn't find a valid cell
		if (next_move == UNKNOWN)
		{
			// while stack is not empty
			// lockInterruptDisable_Gyro_Delay();
			while (upst->index != 0)
			{
				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				// get the cell to test from the stack
				// debug[20] = upst->index;
				next = pop_stack(upst);
				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				// find a neighbor cell with distance one less than current
				minusOneNeighbor(dm, wm, &next, upst, a);

				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			// get next cell to traverse to
			// next_move is actually the direction we need to go next
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			next_move = minusOneNeighbor(dm, wm, c, upst, a);
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		}

		// lockInterruptEnable_Gyro_Delay();
		// Move to next cell
		// First turn to face the correct direction
		int difference = direction - next_move;
		switch (difference)
		{
		case -3:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();

			if (ReCalibFlag && !RemarkAfterTurnMode)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// calibration because left turn backs up mouse a little bit

			break;
		case -2:
			// backward180StillTurn();
			// AML_MotorControl_BackStillTurn();
			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			// calibration by backing into wall behind us
			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(500);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(200);

				// custom_delay(2000);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				// AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			else if (wm->cells[c->x][c->y].walls[direction] == 0 && RemarkAfterBackwardMode == 1)
			{
				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				AML_Encoder_ResetLeftValue();
				while (AML_Encoder_GetLeftValue() > -800)
				{
				}
				AML_MotorControl_ShortBreak('B');
				AML_Encoder_ResetLeftValue();
			}
			break;
		case -1:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();

			if (ReCalibFlag && !RemarkAfterTurnMode)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// uncontrolledAdvanceTicks(450);
			break;
		case 0:
			break;
		case 1:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();

			if (ReCalibFlag && !RemarkAfterTurnMode)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// uncontrolledAdvanceTicks(450);
			break;
		case 2:
			// backward180StillTurn();

			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(200);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				// custom_delay(2000);
				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(250);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			else if (wm->cells[c->x][c->y].walls[direction] == 0 && RemarkAfterBackwardMode == 1)
			{
				AML_MotorControl_LeftPWM(-16);
				AML_MotorControl_RightPWM(-16);
				AML_Encoder_ResetLeftValue();
				while (AML_Encoder_GetLeftValue() > -800)
				{
				}
				AML_MotorControl_ShortBreak('B');
				AML_Encoder_ResetLeftValue();
			}
			break;
		case 3:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();

			if (ReCalibFlag && !RemarkAfterTurnMode)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}
			// uncontrolledAdvanceTicks(450);
			break;
		default:

			// turnOnLEDS();
			break;
		}

		// update the direction we are currently facing
		direction = next_move;
	}
}

int floodFill2(struct dist_maze *dm, struct coor *c, struct wall_maze *wm, int a, int direction, struct stack *upst)
{
	// Disable tracking interrupts because we do not want to move yet
	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	// coordinate for future use in popping stack
	int next_move;
	struct coor next;

	uint16_t TempTicks = 0;
	uint16_t ReCalibTicksFlag = 0;

	// set the base speed of traversal
	// setBaseSpeed(40);
	// AML_MotorControl_SetMouseSpeed(40);

	// while we are not at target destination
	while (1)
	{
		// uint8_t WallFlag = wm->cells[c->x][c->y].walls[direction];

		// update coordinates for next cell we are going to visit
		switch (direction)
		{
		case NORTH:
			c->y += 1;
			break;
		case EAST:
			c->x += 1;
			break;
		case SOUTH:
			c->y -= 1;
			break;
		case WEST:
			c->x -= 1;
			break;
		default:
			break;
		}

		// If we haven't visited the next cell
		if (wm->cells[c->x][c->y].visited == 1)
		{
			// advance one cell
			advanceOneCell(direction, c, wm);

			// check for wall straight ahead
			// if (getLeftADCValue() >= FLOOD_WALL_IN_FRONT_LEFT &&
			// 	getRightADCValue() >= FLOOD_WALL_IN_FRONT_RIGHT)
			if (AML_LaserSensor_ReadSingleWithFillter(FF) < WALL_IN_FRONT)
			{
				// put wall ahead of us
				wm->cells[c->x][c->y].walls[direction] = 1;
				switch (direction)
				{
				// put wall ahead of us in the cell ahead of us
				case NORTH:
					if (c->y + 1 < 16)
						wm->cells[c->x][c->y + 1].walls[SOUTH] = 1;

					break;
				case EAST:
					if (c->x + 1 < 16)
						wm->cells[c->x + 1][c->y].walls[WEST] = 1;

					break;
				case SOUTH:
					if (c->y - 1 > -1)
						wm->cells[c->x][c->y - 1].walls[NORTH] = 1;

					break;
				case WEST:
					if (c->x - 1 > -1)
						wm->cells[c->x - 1][c->y].walls[EAST] = 1;

					break;
				}
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			}
			else
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

				// update the current cell as visited
				wm->cells[c->x][c->y].visited = 1;
		}
		else
		{
			// advance one cell without scanning for walls
			advanceOneCellVisited();
		}

		// if we are at target destination
		if (dm->distance[c->x][c->y] == 0)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			// AML_MotorControl_Stop();
			AML_MotorControl_ShortBreak('F');

			return direction;
			break;
		}

		// check if there is a neighbor with one less distance
		// next_move is the direction we should move next
		next_move = minusOneNeighbor(dm, wm, c, upst, a);

		// If we couldn't find a valid cell
		if (next_move == UNKNOWN)
		{
			// while stack is not empty
			// lockInterruptDisable_Gyro_Delay();
			while (upst->index != 0)
			{
				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				// get the cell to test from the stack
				// debug[20] = upst->index;
				next = pop_stack(upst);
				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				// find a neighbor cell with distance one less than current
				minusOneNeighbor(dm, wm, &next, upst, a);

				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			// get next cell to traverse to
			// next_move is actually the direction we need to go next
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			next_move = minusOneNeighbor(dm, wm, c, upst, a);
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		}

		// lockInterruptEnable_Gyro_Delay();
		// Move to next cell
		// First turn to face the correct direction
		int difference = direction - next_move;
		switch (difference)
		{
		case -3:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();
			uncontrolledAdvanceTicks(AfterTurnTicks);

			// calibration because left turn backs up mouse a little bit

			break;
		case -2:
			// backward180StillTurn();
			// AML_MotorControl_BackStillTurn();
			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			// calibration by backing into wall behind us
			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(500);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(200);

				// custom_delay(2000);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				// AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			break;
		case -1:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();
			uncontrolledAdvanceTicks(AfterTurnTicks);

			// uncontrolledAdvanceTicks(450);
			break;
		case 0:
			break;
		case 1:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();
			uncontrolledAdvanceTicks(AfterTurnTicks);

			// uncontrolledAdvanceTicks(450);
			break;
		case 2:
			// backward180StillTurn();

			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(200);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				// custom_delay(2000);
				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(250);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			break;
		case 3:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();
			uncontrolledAdvanceTicks(AfterTurnTicks);

			// uncontrolledAdvanceTicks(450);
			break;
		default:

			// turnOnLEDS();
			break;
		}

		// update the direction we are currently facing
		direction = next_move;
	}
}

void checkForWalls(struct wall_maze *wm, struct coor *c, int direction, int n, int e, int s, int w)
{
	// if there is a wall in memory
	if (wm->cells[c->x][c->y].walls[w] == 1)
	{
		// check for wall to the west
		// if at anytime the value drops below that means there is no wall
		// if (getLeftFrontADCValue() < FLOOD_LEFT_WALL)
		if (AML_LaserSensor_ReadSingleWithFillter(BL) > WALL_NOT_IN_LEFT)
		{
			wm->cells[c->x][c->y].walls[w] = 0;
			switch (direction)
			{
			// also put walls in the other adjacent cells
			case NORTH:
				if (c->x - 1 > -1)
					wm->cells[c->x - 1][c->y].walls[EAST] = 0;
				break;
			case EAST:
				if (c->y + 1 < 16)
					wm->cells[c->x][c->y + 1].walls[SOUTH] = 0;
				break;
			case SOUTH:
				if (c->x + 1 < 16)
					wm->cells[c->x + 1][c->y].walls[WEST] = 0;
				break;
			case WEST:
				if (c->y - 1 > -1)
					wm->cells[c->x][c->y - 1].walls[NORTH] = 0;
				break;
			}
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		// else
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	}

	if (wm->cells[c->x][c->y].walls[e] == 1)
	{
		// check for wall to the east
		// if (getRightFrontADCValue() < FLOOD_RIGHT_WALL)
		if (AML_LaserSensor_ReadSingleWithFillter(BR) > WALL_NOT_IN_RIGHT)
		{
			wm->cells[c->x][c->y].walls[e] = 0;
			switch (direction)
			{
			case NORTH:
				if (c->x + 1 < 16)
					wm->cells[c->x + 1][c->y].walls[WEST] = 0;
				break;
			case EAST:
				if (c->y - 1 > -1)
					wm->cells[c->x][c->y - 1].walls[NORTH] = 0;
				break;
			case SOUTH:
				if (c->x - 1 > -1)
					wm->cells[c->x - 1][c->y].walls[EAST] = 0;
				break;
			case WEST:
				if (c->y + 1 < 16)
					wm->cells[c->x][c->y + 1].walls[SOUTH] = 0;
				break;
			}
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		// else
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	}
}

int minusOneNeighbor(struct dist_maze *dm, struct wall_maze *wm, struct coor *c, struct stack *s, int a)
{
	int i;
	// minimum distance
	int md = 260;
	// get target distance we're looking for
	int target = dm->distance[c->x][c->y] - 1;
	// check neighbor cells
	for (i = 0; i < 4; i++)
	{
		// choice of direction preference
		int j = (i + a) % 4;
		// If there is no wall blocking the way
		if (wm->cells[c->x][c->y].walls[j] == 0)
		{
			switch (j)
			{
			case NORTH:
				if (dm->distance[c->x][c->y + 1] == target)
				{
					// if the cell exists return the direction we want to move
					return j;
				}
				if (dm->distance[c->x][c->y + 1] < md)
					md = dm->distance[c->x][c->y + 1];
				break;
			case EAST:
				if (dm->distance[c->x + 1][c->y] == target)
				{
					// if the cell exists return the direction we want to move
					return j;
				}
				if (dm->distance[c->x + 1][c->y] < md)
					md = dm->distance[c->x + 1][c->y];
				break;
			case SOUTH:
				if (dm->distance[c->x][c->y - 1] == target)
				{
					// if the cell exists return the direction we want to move
					return j;
				}
				if (dm->distance[c->x][c->y - 1] < md)
					md = dm->distance[c->x][c->y - 1];
				break;
			case WEST:
				if (dm->distance[c->x - 1][c->y] == target)
				{
					// if the cell exists return the direction we want to move
					return j;
				}
				if (dm->distance[c->x - 1][c->y] < md)
					md = dm->distance[c->x - 1][c->y];
				break;
			default:

				break;
			}
		}
	}

	// update distance of coordinate to 1 plus minimum distance
	dm->distance[c->x][c->y] = md + 1;

	// Since we did not find a cell we push onto the stack
	for (i = 0; i < 4; i++)
	{
		// choice of direction preference
		int j = (i + a) % 4;
		// If there is no wall blocking the way
		if (wm->cells[c->x][c->y].walls[j] == 0)
		{
			struct coor temp;
			switch (j)
			{
			case NORTH:
				init_coor(&temp, c->x, c->y + 1);
				break;
			case EAST:
				init_coor(&temp, c->x + 1, c->y);
				break;
			case SOUTH:
				init_coor(&temp, c->x, c->y - 1);
				break;
			case WEST:
				init_coor(&temp, c->x - 1, c->y);
				break;
			}
			// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

			// push_stack(s, temp);

			if (temp.x >= 0 && temp.x < 16 && temp.y >= 0 && temp.y < 16)
			{
				push_stack(s, temp);
			}

			// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}
	// return unknown
	return UNKNOWN;
}

// Show the coordinates
void showCoor(int x, int y)
{
	// initialize coordinates binary representation
	int binaryx[4];
	int binaryy[4];
	int xcoor = x;
	int ycoor = y;

	// get the binary values
	for (int i = 3; i > -1; i--)
	{
		if (xcoor - (1 << i) >= 0)
		{
			binaryx[i] = 1;
			xcoor -= (1 << i);
		}
		else
			binaryx[i] = 0;

		if (ycoor - (1 << i) >= 0)
		{
			binaryy[i] = 1;
			ycoor -= (1 << i);
		}
		else
			binaryy[i] = 0;
	}

	// blink leds to show binary values
	// turnOffCenterLEDS();

	// if (binaryx[3])
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	// if (binaryx[2])
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	// if (binaryx[1])
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	// if (binaryx[0])
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	// HAL_Delay(500);
	// turnOffCenterLEDS();
	// if (binaryy[3])
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	// if (binaryy[2])
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	// if (binaryy[1])
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	// if (binaryy[0])
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	// HAL_Delay(500);
	// turnOffCenterLEDS();
}

// void turnOnCenterLEDS()
// {
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
// }

// void turnOffCenterLEDS()
// {
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
// }

void advanceOneCell(int direction, struct coor *c, struct wall_maze *wm)
{
	// go forward one cell
	// lockInterruptEnable_TIM3();
	AML_MotorControl_TurnOnWallFollow();

	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	AML_DebugDevice_TurnOnLED(0);

	// advanceTicksFlood(FLOOD_ONE_CELL, direction, c, wm);
	
	advanceTicksFlood(MAZE_ENCODER_TICKS_ONE_CELL, direction, c, wm);

	AML_DebugDevice_BuzzerBeep(20);

	AML_MotorControl_TurnOffWallFollow();

	// lockInterruptDisable_TIM3();

	// motorStop();
	// AML_MotorControl_Stop();

	if (DebugMode)
	{
		AML_MotorControl_ShortBreak('F');

		// custom_delay(500);
		HAL_Delay(1000);
	}

	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	AML_DebugDevice_TurnOffLED(0);

	// resetLeftEncoder();
	AML_Encoder_ResetLeftValue();
}

void advanceOneCellVisited()
{
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	AML_DebugDevice_TurnOnLED(1);

	// Go forward one cell
	// lockInterruptEnable_TIM3();
	AML_DebugDevice_BuzzerBeep(20);

	AML_MotorControl_TurnOnWallFollow();

	uncontrolledAdvanceTicks(MAZE_ENCODER_TICKS_ONE_CELL);

	// debug[15]++;

	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	// motorStop();
	// AML_MotorControl_Stop();
	if (DebugMode)
	{
		AML_MotorControl_ShortBreak('F');

		// custom_delay(500);
		HAL_Delay(1000);
	}

	// showCoor(c.x, c.y);

	AML_DebugDevice_TurnOffLED(1);
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	// resetLeftEncoder();
	AML_Encoder_ResetLeftValue();
}

int centerMovement(struct wall_maze *wm, struct coor *c, int d)
{
	// 1 for turn right. 0 for turn left
	int turn = 0;
	//	switch(d)
	//	{
	//	case NORTH:
	//		if(c->x==7 && c->y==7) turn = 1;
	//		break;
	//	case EAST:
	//		if(c->x==7 && c->y==8) turn = 1;
	//		break;
	//	case SOUTH:
	//		if(c->x==8 && c->y==8) turn = 1;
	//		break;
	//	case WEST:
	//		if(c->x==8 && c->y==7) turn = 1;
	//		break;
	//	}
	//
	//	for(int i=0 ; i<4 ; i++)
	//	{
	//		switch(d)
	//		{
	//		case NORTH: c->y += 1;
	//		break;
	//		case EAST: c->x += 1;
	//		break;
	//		case SOUTH: c->y -= 1;
	//		break;
	//		case WEST: c->x -= 1;
	//		break;
	//		default:
	//			break;
	//		}
	//
	//		advanceOneCell(d, c, wm);
	//		// showCoor(c.x, c.y);
	//
	//		// check for wall straight ahead
	//		if(getLeftADCValue() >= FLOOD_WALL_IN_FRONT_LEFT &&
	//				getRightADCValue() >= FLOOD_WALL_IN_FRONT_RIGHT)
	//		{
	//			wm->cells[c->x][c->y].walls[d] = 1;
	//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	//		}
	//		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	//		// update the current cell as visited
	//		wm->cells[c->x][c->y].visited = 1;
	//
	//		if(turn)
	//		{
	//			rightStillTurn();
	//			switch(d)
	//			{
	//			case NORTH: d = EAST;
	//			break;
	//			case EAST: d = SOUTH;
	//			break;
	//			case SOUTH: d = WEST;
	//			break;
	//			case WEST: d = NORTH;
	//			break;
	//			default:
	//				break;
	//			}
	//		}
	//		else
	//		{
	//			leftStillTurn();
	//			switch(d)
	//			{
	//			case NORTH: d = WEST;
	//			break;
	//			case EAST: d = NORTH;
	//			break;
	//			case SOUTH: d = EAST;
	//			break;
	//			case WEST: d = SOUTH;
	//			break;
	//			default:
	//				break;
	//			}
	//		}
	//	}
	//	return d;
	int direction;
	// put down all the walls in the center depending on which cell we entered
	// and also the direction we are currently facing
	switch (d)
	{
	case NORTH:
		if (c->x == 7 && c->y == 7)
		{
			wm->cells[c->x + 1][c->y].walls[SOUTH] = 1;
			wm->cells[c->x + 1][c->y].walls[EAST] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[NORTH] = 1;
			wm->cells[c->x + 2][c->y].walls[WEST] = 1;
			wm->cells[c->x][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x][c->y + 1].walls[WEST] = 1;
			wm->cells[c->x][c->y + 2].walls[SOUTH] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x + 1][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x + 1][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x + 1][c->y + 2].walls[SOUTH] = 1;
			wm->cells[c->x + 2][c->y + 1].walls[WEST] = 1;
		}
		else
		{
			wm->cells[c->x - 1][c->y].walls[SOUTH] = 1;
			wm->cells[c->x - 1][c->y].walls[WEST] = 1;
			wm->cells[c->x][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[WEST] = 1;
		}
		direction = SOUTH;
		break;
	case EAST:
		if (c->x == 7 && c->y == 7)
		{
			wm->cells[c->x + 1][c->y].walls[SOUTH] = 1;
			wm->cells[c->x + 1][c->y].walls[EAST] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x][c->y + 2].walls[SOUTH] = 1;
			wm->cells[c->x][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x][c->y + 1].walls[WEST] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[NORTH] = 1;
			wm->cells[c->x + 2][c->y].walls[EAST] = 1;
			wm->cells[c->x + 1][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x + 1][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x + 1][c->y + 2].walls[SOUTH] = 1;
			wm->cells[c->x + 2][c->y + 1].walls[EAST] = 1;
		}
		else
		{
			wm->cells[c->x + 1][c->y].walls[NORTH] = 1;
			wm->cells[c->x + 1][c->y].walls[EAST] = 1;
			wm->cells[c->x][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x][c->y - 1].walls[WEST] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[EAST] = 1;
		}
		direction = WEST;
		break;
	case SOUTH:
		if (c->x == 7 && c->y == 8)
		{
			wm->cells[c->x + 1][c->y].walls[NORTH] = 1;
			wm->cells[c->x + 1][c->y].walls[EAST] = 1;
			wm->cells[c->x][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x][c->y - 1].walls[WEST] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x + 1][c->y - 1].walls[EAST] = 1;
		}
		else
		{
			wm->cells[c->x - 1][c->y].walls[NORTH] = 1;
			wm->cells[c->x - 1][c->y].walls[WEST] = 1;
			wm->cells[c->x][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x][c->y - 1].walls[EAST] = 1;
			wm->cells[c->x - 1][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x - 1][c->y - 1].walls[WEST] = 1;
		}
		direction = NORTH;
		break;
	case WEST:
		if (c->x == 8 && c->y == 8)
		{
			wm->cells[c->x - 1][c->y].walls[NORTH] = 1;
			wm->cells[c->x - 1][c->y].walls[WEST] = 1;
			wm->cells[c->x][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x][c->y - 1].walls[EAST] = 1;
			wm->cells[c->x - 1][c->y - 1].walls[SOUTH] = 1;
			wm->cells[c->x - 1][c->y - 1].walls[WEST] = 1;
		}
		else
		{
			wm->cells[c->x - 1][c->y].walls[SOUTH] = 1;
			wm->cells[c->x - 1][c->y].walls[WEST] = 1;
			wm->cells[c->x][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x][c->y + 1].walls[EAST] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[NORTH] = 1;
			wm->cells[c->x - 1][c->y + 1].walls[WEST] = 1;
		}
		direction = EAST;
		break;
	}
	// all center cells are visited
	wm->cells[7][7].visited = 1;
	wm->cells[7][8].visited = 1;
	wm->cells[8][7].visited = 1;
	wm->cells[8][8].visited = 1;
	// recalibrate on center wall, then drive out
	advanceOneCellVisited();

	// backward180StillTurn();
	// AML_MotorControl_TurnLeft180();

	if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
	{
		AML_MotorControl_TurnLeft180();
	}
	else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
	{
		AML_MotorControl_TurnRight180();
	}
	else
	{
		AML_MotorControl_TurnLeft180();
	}

	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	// leftMotorPWMChangeBackward(200);
	// rightMotorPWMChangeBackward(200);
	// custom_delay(2000);

	// di lui de ap sat tuong
	AML_MotorControl_LeftPWM(-20);
	AML_MotorControl_RightPWM(-20);
	HAL_Delay(1000);

	AML_MPUSensor_ResetAngle();

	// motorStop();
	AML_MotorControl_Stop();

	// lockInterruptEnable_TIM3();
	AML_MotorControl_TurnOnWallFollow();

	// uncontrolledAdvanceTicks(3000);
	// uncontrolledAdvanceTicks(200);

	advanceOneCellVisited();

	return direction;
}

int logicalFlood(struct dist_maze *dm, struct coor *c, struct wall_maze *wm, int a, int direction, struct stack *upst)
{
	// Disable tracking interrupts because we do not want to move yet
	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	int next_move;
	struct coor next;
	int x = c->x;
	int y = c->y;
	// while we are not at target destination
	while (1)
	{
		// AML_MotorControl_GoStraight();

		// update coordinates for next cell we are going to visit
		switch (direction)
		{
		case NORTH:
			c->y += 1;
			break;
		case EAST:
			c->x += 1;
			break;
		case SOUTH:
			c->y -= 1;
			break;
		case WEST:
			c->x -= 1;
			break;
		default:
			break;
		}

		// return if we have reached our target
		if (dm->distance[c->x][c->y] == 0)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			// AML_MotorControl_ShortBreak('F');

			// reassign the previous coordinates before logical flood
			c->x = x;
			c->y = y;
			return direction;
			break;
		}

		// check if there is a neighbor with one less distance
		// next_move is the direction we should move next
		next_move = minusOneNeighbor(dm, wm, c, upst, a);

		// If we couldn't find a valid cell
		if (next_move == UNKNOWN)
		{
			// while stack is not empty
			// lockInterruptDisable_Gyro_Delay();
			while (upst->index != 0)
			{
				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

				// get the cell to test from the stack
				next = pop_stack(upst);

				// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				// find a neighbor cell with distance one less than current
				minusOneNeighbor(dm, wm, &next, upst, a);

				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}

			// get next cell to traverse to
			// next_move is actually the direction we need to go next
			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

			next_move = minusOneNeighbor(dm, wm, c, upst, a);

			// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		}

		// lockInterruptEnable_Gyro_Delay();

		// update the direction we are currently facing
		direction = next_move;
	}
}

void shortestPath(struct dist_maze *dm, struct coor *c, struct wall_maze *wm, int a, int direction, struct stack *upst)
{
	// Disable tracking interrupts because we do not want to move yet
	// lockInterruptDisable_TIM3();
	AML_MotorControl_TurnOffWallFollow();

	// coordinate for future use in popping stack
	int next_move = 0;
	int d;
	d = direction;

	uint8_t ReCalibFlag = 0;

	// dm->distance[0][0] = 250;

	// set the base traversal speed
	// setBaseSpeed(40);

	// while not at target destination
	while (1)
	{
		// update coordinates
		switch (d)
		{
		case NORTH:
			c->y += 1;
			break;
		case EAST:
			c->x += 1;
			break;
		case SOUTH:
			c->y -= 1;
			break;
		case WEST:
			c->x -= 1;
			break;
		default:
			break;
		}
		// move forward one cell
		advanceOneCellVisited();

		// if we are at the center exit function
		if (c->x == 7 && c->y == 7)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			AML_MotorControl_ShortBreak('F');
			break;
		}
		if (c->x == 7 && c->y == 8)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			AML_MotorControl_ShortBreak('F');
			break;
		}
		if (c->x == 8 && c->y == 7)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			AML_MotorControl_ShortBreak('F');
			break;
		}
		if (c->x == 8 && c->y == 8)
		{
			// lockInterruptDisable_TIM3();
			AML_MotorControl_TurnOffWallFollow();
			AML_MotorControl_ShortBreak('F');
			break;
		}

		// find the target which is the distance of current cell plus one
		int target = dm->distance[c->x][c->y] - 1;

		// check neighbor cells
		for (int i = 0; i < 4; i++)
		{
			switch (i)
			{
			case NORTH:
				if (c->y + 1 < 16)
				{
					if (dm->distance[c->x][c->y + 1] == target && wm->cells[c->x][c->y + 1].visited == 1 && wm->cells[c->x][c->y].walls[i] == 0)
					{
						// if the cell exists return the direction we want to move
						next_move = i;
					}
				}
				break;
			case EAST:
				if (c->x + 1 < 16)
				{
					if (dm->distance[c->x + 1][c->y] == target && wm->cells[c->x + 1][c->y].visited == 1 && wm->cells[c->x][c->y].walls[i] == 0)
					{
						// if the cell exists return the direction we want to move
						next_move = i;
					}
				}
				break;
			case SOUTH:
				if (c->y - 1 > -1)
				{
					if (dm->distance[c->x][c->y - 1] == target && wm->cells[c->x][c->y - 1].visited == 1 && wm->cells[c->x][c->y].walls[i] == 0)
					{
						// if the cell exists return the direction we want to move
						next_move = i;
					}
				}
				break;
			case WEST:
				if (c->x - 1 > -1)
				{
					if (dm->distance[c->x - 1][c->y] == target && wm->cells[c->x - 1][c->y].visited == 1 && wm->cells[c->x][c->y].walls[i] == 0)
					{
						// if the cell exists return the direction we want to move
						next_move = i;
					}
				}
				break;
			default:
				break;
			}
		}

		// find the direction we must turn to
		int difference = d - next_move;
		switch (difference)
		{
		case -3:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();

			if (ReCalibFlag)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// calibration because left turn backs up mouse a little bit

			break;
		case -2:
			// backward180StillTurn();
			// AML_MotorControl_BackStillTurn();
			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			// calibration by backing into wall behind us
			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(500);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(200);

				// custom_delay(2000);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				// AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			break;
		case -1:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();

			if (ReCalibFlag)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// uncontrolledAdvanceTicks(450);
			break;
		case 0:
			break;
		case 1:
			// leftStillTurn();
			// AML_MotorControl_LeftStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnLeft90();

			if (ReCalibFlag)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}

			// uncontrolledAdvanceTicks(450);
			break;
		case 2:
			// backward180StillTurn();

			// AML_MotorControl_TurnLeft180();

			if (AML_LaserSensor_ReadSingleWithoutFillter(BL) > WALL_NOT_IN_LEFT)
			{
				AML_MotorControl_TurnLeft180();
			}
			else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) > WALL_NOT_IN_RIGHT)
			{
				AML_MotorControl_TurnRight180();
			}
			else
			{
				AML_MotorControl_TurnLeft180();
			}

			if (wm->cells[c->x][c->y].walls[direction] == 1)
			{
				// lockInterruptDisable_TIM3();
				// uncontrolledAdvanceTicks(200);
				AML_MotorControl_TurnOffWallFollow();

				// leftMotorPWMChangeBackward(200);
				// rightMotorPWMChangeBackward(200);

				// custom_delay(2000);
				AML_MotorControl_LeftPWM(-20);
				AML_MotorControl_RightPWM(-20);
				HAL_Delay(1000);

				AML_MPUSensor_ResetAngle();
				HAL_Delay(100);

				uncontrolledAdvanceTicks(AfterTurnTicks);

				// uncontrolledAdvanceTicks(250);

				// motorStop();
				AML_MotorControl_Stop();

				// lockInterruptEnable_TIM3();
				AML_MotorControl_TurnOnWallFollow();

				// uncontrolledAdvanceTicks(3000);
			}
			break;
		case 3:
			// rightStillTurn();
			// AML_MotorControl_RightStillTurn();
			if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
			{
				ReCalibFlag = 1;
			}
			else
			{
				ReCalibFlag = 0;
			}

			if (AML_LaserSensor_ReadSingleWithoutFillter(FF) > WALL_NOT_IN_FRONT)
			{
				uncontrolledAdvanceTicks(BeforeTurnTicks);
				AML_MotorControl_ShortBreak('F');
			}
			else
			{
				uncontrolledAdvanceTicks(500);
			}

			AML_MotorControl_TurnRight90();

			if (ReCalibFlag)
			{
				uncontrolledAdvanceTicks(AfterTurnTicks);
			}
			// uncontrolledAdvanceTicks(450);
			break;
		default:

			// turnOnLEDS();
			break;
		}

		// update the direction we are currently facing
		d = next_move;
	}
}
