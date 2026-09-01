# Motor CAN Enable/Disable Commands

## Damiao Motors (IDs 10, 12, 16, 14)

### Enable/Disable (sent to base CAN ID, all modes)

| Motor ID (dec) | Motor ID (hex) |
|---|---|
| 10 | 0x0A |
| 12 | 0x0C |
| 16 | 0x10 |
| 14 | 0x0E |


**Enable:**
```bash
cansend can0 00A#FFFFFFFFFFFFFFFC
cansend can0 00C#FFFFFFFFFFFFFFFC
cansend can0 010#FFFFFFFFFFFFFFFC
cansend can0 00E#FFFFFFFFFFFFFFFC
```

**Disable:**
```bash
cansend can0 00A#FFFFFFFFFFFFFFFD
cansend can0 00C#FFFFFFFFFFFFFFFD
cansend can0 010#FFFFFFFFFFFFFFFD
cansend can0 00E#FFFFFFFFFFFFFFFD
```

## SteadyWin Motors (IDs 11, 13, 17, 15)

These motors do not have separate enbale command. So any commands thet says to move to some position works as enable

**Enable/move to 0 position:**
```bash
cansend can0 00B#C200000000
cansend can0 00D#C200000000
cansend can0 011#C200000000
cansend can0 00F#C200000000
```

**Disable:**
```bash
cansend can0 00B#CF
cansend can0 00D#CF
cansend can0 011#CF
cansend can0 00F#CF
```

**Set motors zero position:**

```bash
# Set origin for Motor ID 11 (0x0B)      
cansend can0 00B#B1

# Set origin for Motor ID 13 (0x0D)
cansend can0 00D#B1

# Set origin for Motor ID 15 (0x0F)
cansend can0 00F#B1

# Set origin for Motor ID 17 (0x11)
cansend can0 011#B1
```


## Sources

- `Custom_CAN_communication_protocol_V3_06b0.pdf` (Damiao custom protocol, `0xCF` disable command)
- `DAMIAO-DM-J10010L-2EC-User_Manual` (MIT mode control frame format)
