const { deviceAddCustomCluster, binary, numeric, onOff, illuminance } = require('zigbee-herdsman-converters/lib/modernExtend');
const Zcl = require('zigbee-herdsman').Zcl;
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = require('zigbee-herdsman-converters/lib/exposes');

// Fonctions pour les attributs de la cible 1
function target1_presence() {
    return e.binary('target1_presence', e.access.STATE_GET, true, false)
        .withDescription('Target 1 presence state')
        .withProperty('target1_presence');
}

function target1_distance() {
    return e.numeric('target1_distance', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 1 distance in mm')
        .withProperty('target1_distance');
}

function target1_x() {
    return e.numeric('target1_x', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 1 X coordinate in mm')
        .withProperty('target1_x');
}

function target1_y() {
    return e.numeric('target1_y', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 1 Y coordinate in mm')
        .withProperty('target1_y');
}

// Fonctions pour les attributs de la cible 2
function target2_presence() {
    return e.binary('target2_presence', e.access.STATE_GET, true, false)
        .withDescription('Target 2 presence state')
        .withProperty('target2_presence');
}

function target2_distance() {
    return e.numeric('target2_distance', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 2 distance in mm')
        .withProperty('target2_distance');
}

function target2_x() {
    return e.numeric('target2_x', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 2 X coordinate in mm')
        .withProperty('target2_x');
}

function target2_y() {
    return e.numeric('target2_y', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 2 Y coordinate in mm')
        .withProperty('target2_y');
}

// Fonctions pour les attributs de la cible 3
function target3_presence() {
    return e.binary('target3_presence', e.access.STATE_GET, true, false)
        .withDescription('Target 3 presence state')
        .withProperty('target3_presence');
}

function target3_distance() {
    return e.numeric('target3_distance', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 3 distance in mm')
        .withProperty('target3_distance');
}

function target3_x() {
    return e.numeric('target3_x', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 3 X coordinate in mm')
        .withProperty('target3_x');
}

function target3_y() {
    return e.numeric('target3_y', e.access.STATE_GET)
        .withUnit('mm')
        .withDescription('Target 3 Y coordinate in mm')
        .withProperty('target3_y');
}

// Convertisseur personnalisé pour le cluster radar
const fromZigbeeRadar = {
    cluster: 'radar',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        console.log('[SQUWAL] ===== CONVERTISSEUR RADAR APPELÉ =====');
        console.log('[SQUWAL] Message radar reçu:', JSON.stringify(msg));
        console.log('[SQUWAL] Message data:', JSON.stringify(msg.data));
        
        // Créer un objet pour stocker les résultats
        const result = {};
        
        // Traiter les attributs reçus
        if ('presence_target_1' in msg.data) {
            console.log('[SQUWAL] Présence détectée dans les données:', msg.data.presence_target_1);
            // Convertir explicitement en booléen
            result.target1_presence = (msg.data.presence_target_1 === 1 || msg.data.presence_target_1 === true);
            console.log('[SQUWAL] Valeur convertie pour target1_presence:', result.target1_presence);
        }
        
        if ('distance_target_1' in msg.data) {
            console.log('[SQUWAL] Distance détectée dans les données:', msg.data.distance_target_1);
            result.target1_distance = msg.data.distance_target_1;
        }
        
        if ('coord_x_target_1' in msg.data) {
            result.target1_x = msg.data.coord_x_target_1;
        }
        
        if ('coord_y_target_1' in msg.data) {
            result.target1_y = msg.data.coord_y_target_1;
        }
        
        // Cible 2
        if ('presence_target_2' in msg.data) {
            result.target2_presence = (msg.data.presence_target_2 === 1 || msg.data.presence_target_2 === true);
        }
        
        if ('distance_target_2' in msg.data) {
            result.target2_distance = msg.data.distance_target_2;
        }
        
        if ('coord_x_target_2' in msg.data) {
            result.target2_x = msg.data.coord_x_target_2;
        }
        
        if ('coord_y_target_2' in msg.data) {
            result.target2_y = msg.data.coord_y_target_2;
        }
        
        // Cible 3
        if ('presence_target_3' in msg.data) {
            result.target3_presence = (msg.data.presence_target_3 === 1 || msg.data.presence_target_3 === true);
        }
        
        if ('distance_target_3' in msg.data) {
            result.target3_distance = msg.data.distance_target_3;
        }
        
        if ('coord_x_target_3' in msg.data) {
            result.target3_x = msg.data.coord_x_target_3;
        }
        
        if ('coord_y_target_3' in msg.data) {
            result.target3_y = msg.data.coord_y_target_3;
        }
        
        console.log('[SQUWAL] Résultat traité:', JSON.stringify(result));
        return result;
    },
};

const definition = {
    zigbeeModel: ['ESP32-C6 LD2450', 'ESP-C6 Presency', 'esp32c6'],
    model: 'ESP32-C6 LD2450',
    vendor: 'SQUWAL INC',
    description: 'ESP32-C6 LD2450 Presence Sensor',
    fromZigbee: [fromZigbeeRadar],
    toZigbee: [],
    exposes: [
        // Attributs de la cible 1
        target1_presence(),
        target1_distance(),
        target1_x(),
        target1_y(),
        // Attributs de la cible 2
        target2_presence(),
        target2_distance(),
        target2_x(),
        target2_y(),
        // Attributs de la cible 3
        target3_presence(),
        target3_distance(),
        target3_x(),
        target3_y(),

    ],
    endpoint: (device) => {
        return {
            radar: 10,
            led: 10,
            illuminance: 10
        };
    },
    extend: [
        deviceAddCustomCluster('radar', {
            ID: 0xff00,
            attributes: {
                presence_target_1: { ID: 0x0000, type: Zcl.DataType.BOOLEAN },
                distance_target_1: { ID: 0x0001, type: Zcl.DataType.UINT16 },
                coord_x_target_1: { ID: 0x0002, type: Zcl.DataType.UINT16 },
                coord_y_target_1: { ID: 0x0003, type: Zcl.DataType.UINT16 },
                presence_target_2: { ID: 0x0010, type: Zcl.DataType.BOOLEAN },
                distance_target_2: { ID: 0x0011, type: Zcl.DataType.UINT16 },
                coord_x_target_2: { ID: 0x0012, type: Zcl.DataType.UINT16 },
                coord_y_target_2: { ID: 0x0013, type: Zcl.DataType.UINT16 },
                presence_target_3: { ID: 0x0020, type: Zcl.DataType.BOOLEAN },
                distance_target_3: { ID: 0x0021, type: Zcl.DataType.UINT16 },
                coord_x_target_3: { ID: 0x0022, type: Zcl.DataType.UINT16 },
                coord_y_target_3: { ID: 0x0023, type: Zcl.DataType.UINT16 }
            },
            commands: {},
            commandsResponse: {},
        }),
        // Ajout de l'illuminance et du switch
        illuminance(),
        onOff(),
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        console.log('[SQUWAL] Configuration du dispositif ESP32-C6 LD2450');
        
        // Configuration du cluster radar
        const radarEndpoint = device.getEndpoint(10);
        if (radarEndpoint) {
            try {
                console.log('[SQUWAL] Liaison du cluster personnalisé radar (0xFF00)');
                await reporting.bind(radarEndpoint, coordinatorEndpoint, ['radar']);
                console.log('[SQUWAL] Cluster personnalisé lié avec succès');
                
                // Configuration du reporting pour les attributs du cluster personnalisé
                // Paramètres: 
                // - Présence: min = 1 seconde, max = 43200 secondes (12 heures), changement = 1
                // - Distance et coordonnées: min = 30 secondes, max = 43200 secondes (12 heures), changement = 400mm
                console.log('[SQUWAL] Configuration du reporting pour les attributs du cluster personnalisé');
                
                // Target 1
                await radarEndpoint.configureReporting('radar', [
                    {attribute: 'presence_target_1', minimumReportInterval: 1, maximumReportInterval: 43200, reportableChange: 1},
                    {attribute: 'distance_target_1', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_x_target_1', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_y_target_1', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                ]);
                
                // Target 2
                await radarEndpoint.configureReporting('radar', [
                    {attribute: 'presence_target_2', minimumReportInterval: 1, maximumReportInterval: 43200, reportableChange: 1},
                    {attribute: 'distance_target_2', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_x_target_2', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_y_target_2', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                ]);
                
                // Target 3
                await radarEndpoint.configureReporting('radar', [
                    {attribute: 'presence_target_3', minimumReportInterval: 1, maximumReportInterval: 43200, reportableChange: 1},
                    {attribute: 'distance_target_3', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_x_target_3', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                    {attribute: 'coord_y_target_3', minimumReportInterval: 30, maximumReportInterval: 43200, reportableChange: 400},
                ]);
                
                console.log('[SQUWAL] Reporting configuré avec succès pour le cluster personnalisé');
            } catch (e) {
                console.warn(`[SQUWAL] Erreur lors de la configuration du cluster personnalisé: ${e}`);
                console.info('[SQUWAL] Continuez malgré l\'erreur, les messages du cluster personnalisé peuvent toujours être reçus');
            }
        } else {
            console.warn('[SQUWAL] Endpoint radar non trouvé, vérifiez la configuration de votre appareil');
        }
        
        // Configuration des clusters standard
        const endpoint2 = device.getEndpoint(10);
        if (endpoint2) {
            try {
                // Configuration de l'illuminance
                await reporting.bind(endpoint2, coordinatorEndpoint, ['msIlluminanceMeasurement']);
                await reporting.illuminance(endpoint2);
                
                // Configuration du switch
                await reporting.bind(endpoint2, coordinatorEndpoint, ['genOnOff']);
                await reporting.onOff(endpoint2);
                
                console.log('[SQUWAL] Clusters standard configurés avec succès');
            } catch (e) {
                console.warn(`[SQUWAL] Erreur lors de la configuration des clusters standard: ${e}`);
            }
        } else {
            console.warn('[SQUWAL] Endpoint 2 non trouvé, vérifiez la configuration de votre appareil');
        }
    },
};

module.exports = definition;