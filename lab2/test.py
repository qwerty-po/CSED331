import random

class Dimension:
    def __init__(self):
        pass

    def generate(self):
        raise NotImplemented('this is abstract class')

    def emunerate(self):
        raise NotImplemented('this is abstract class')

class EnumDimension(Dimension):
    def __init__(self, enums):
        super().__init__()
        self.enums = enums

    def generate(self):
        index, enum = random.choice(list(enumerate(self.enums)))
        return index, enum

    def emunerate(self):
        raise NotImplemented()

class IntegerDimension(Dimension):
    def __init__(self, bits, signed=True, boundary=0):
        super().__init__()
        self.bits = bits
        self.signed = signed
        self.boundary = boundary

    def generate(self):
        value = random.randint(0, (1 << (self.bits - 1)) - 1)
        value //= 1 << self.boundary
        value *= 1 << self.boundary
        if self.signed and (value & (1 << (self.bits - 2))):
            value = ~value - 1
        return value, value

    def emunerate(self):
        raise NotImplemented()

class ConstantDimension(Dimension):
    def __init__(self, value):
        super().__init__()
        self.value = value

    def generate(self):
        return self.value, self.value

    def emunerate(self):
        raise NotImplemented()

class Location:
    def __init__(self, end, begin, sample_end=None, sample_begin=None):
        self.end = end
        self.begin = begin
        self.sample_end = sample_end or 31
        self.sample_begin = sample_begin or 0

    def into(self, value):
        value >>= self.sample_begin
        value &= (1 << (self.sample_end - self.sample_begin + 1)) - 1
        value &= (1 << (self.end - self.begin + 1)) - 1
        value <<= self.begin
        return value

class CompoundLocation:
    def __init__(self, *locations):
        self.locations = locations
    
    def into(self, value):
        code = 0
        for location in self.locations:
            code |= location.into(value)
        return code

class Instruction:
    def __init__(self, command, opcode, dimensions):
        self.command = command
        self.opcode = opcode
        self.dimensions = dimensions
    
    def generate(self):
        parsed_obj = {}

        command = self.command
        code = self.opcode

        parsed_obj['command'] = command
        parsed_obj['opcode'] = self.opcode

        for name, location, dimension in self.dimensions:      
            value, expression = dimension.generate()
            if name != 'funct':
                if '(' not in command.split(' ')[-1]:
                    command += ' '
                command += str(expression)
                if name == '!offset':
                    command += '('
                elif '(' in command.split(' ')[-1]:
                    command += ')'
            code |= location.into(value)
            parsed_obj[name if name != '!offset' else 'imm'] = value

        return code, command, parsed_obj

class InstructionGroup:
    def __init__(self, *instructions):
        self.instructions = instructions
    
    def generate(self):
        instruction = random.choice(self.instructions)
        return instruction.generate()

REGISTERS = ['zero', 'ra', 'sp', 'gp', 'tp', 't0', 't1', 't2', 's0', 's1', *[f'a{x}' for x in range(8)], *[f's{x}' for x in range(2, 12)], *[f't{x}' for x in range(3, 7)]]

def rtype(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', CompoundLocation(Location(31, 25, 9, 3), Location(14, 12, 2, 0)), ConstantDimension(funct)),
        ('rd', Location(11, 7), EnumDimension(REGISTERS)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
        ('rs2', Location(24, 20), EnumDimension(REGISTERS)),
    ])

def itype(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
        ('rd', Location(11, 7), EnumDimension(REGISTERS)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
        ('imm', Location(31, 20), IntegerDimension(12)),
    ])

def itype_load(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
        ('rd', Location(11, 7), EnumDimension(REGISTERS)),
        ('!offset', Location(31, 20), IntegerDimension(12, boundary=2)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
    ])

def itype_unsigned(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
        ('rd', Location(11, 7), EnumDimension(REGISTERS)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
        ('imm', Location(31, 20), IntegerDimension(12, False)),
    ])

def itype_shift(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
        ('rd', Location(11, 7), EnumDimension(REGISTERS)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
        ('imm', Location(24, 20), IntegerDimension(5, False)),
    ])

# def jtype(command, opcode):
#     return Instruction(command, opcode, [
#         ('rd', Location(11, 7), EnumDimension(REGISTERS)),
#         ('imm', CompoundLocation(
#             Location(31, 31, 20, 20),
#             Location(30, 21, 10, 1),
#             Location(20, 20, 11, 11),
#             Location(19, 12, 19, 12),
#         ), IntegerDimension(12, False, boundary=2)),
#     ])

# def btype(command, opcode, funct):
#     return Instruction(command, opcode, [
#         ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
#         ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
#         ('rs2', Location(24, 20), EnumDimension(REGISTERS)),
#         ('imm', CompoundLocation(
#             Location(31, 31, 12, 12),
#             Location(30, 25, 10, 5),
#             Location(11, 8, 4, 1),
#             Location(7, 7, 11, 11),
#         ), IntegerDimension(12, boundary=2)),
#     ])

def stype(command, opcode, funct):
    return Instruction(command, opcode, [
        ('funct', Location(14, 12, 2, 0), ConstantDimension(funct)),
        ('rs2', Location(24, 20), EnumDimension(REGISTERS)),
        ('!offset', CompoundLocation(
            Location(31, 25, 11, 5),
            Location(11, 7, 4, 0),
        ), IntegerDimension(12, boundary=2)),
        ('rs1', Location(19, 15), EnumDimension(REGISTERS)),
    ])

INSTRUCTIONS = InstructionGroup(
    rtype('add', 0b0110011, 0b0000000000),
    rtype('sub', 0b0110011, 0b0100000000),
    rtype('sll', 0b0110011, 0b0000000001),
    rtype('xor', 0b0110011, 0b0000000100),
    rtype('srl', 0b0110011, 0b0000000101),
    rtype('or',  0b0110011, 0b0000000110),
    rtype('and', 0b0110011, 0b0000000111),

    itype('addi', 0b0010011, 0b000),
    itype_unsigned('xori', 0b0010011, 0b100),
    itype_unsigned('ori', 0b0010011, 0b110),
    itype_unsigned('andi', 0b0010011, 0b111),
    itype_shift('slli', 0b0010011, 0b001),
    itype_shift('srli', 0b0010011, 0b101),
    itype_load('lw', 0b0000011, 0b010),

    # jtype('jal', 0b1101111),

    # btype('beq', 0b1100011, 0b000),
    # btype('bne', 0b1100011, 0b001),
    # btype('blt', 0b1100011, 0b100),
    # btype('bge', 0b1100011, 0b101),

    stype('sw', 0b0100011, 0b010),
)

if __name__ == '__main__':
    fp1 = open("asm_file.s", "w")
    fp2 = open("asm_mem.txt", "w")
    fp1.write("<main>: \n")
    for i in range(100):
        code, command, parsed_obj = INSTRUCTIONS.generate()
        if 'a7' in command:
            i-=1
            continue
        if 'lw' in command and '-' in command:
            i-=1
            continue
        fp2.write(hex(code)[2:].zfill(8)+"\n")
        fp1.write("\t"+command+"\n")
        print(hex(code)[2:].zfill(8), '//', command)

    fp1.write("\tecall\n\tli a7, 10\n\tecall\n")
    fp2.write("00000073\n00a00893\n00000073")
    fp1.close()
    fp2.close()