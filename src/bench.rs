extern crate test;
use crate::{Centerline, CenterlineError};
use boostvoronoi::Line;
use test::Bencher;

#[cfg(test)]
#[bench]
fn bench_1(b: &mut Bencher) -> Result<(), CenterlineError> {
    let segments: [[i32; 4]; 352] = [
        [402, 20, 395, 20],
        [408, 23, 402, 20],
        [476, 27, 469, 26],
        [328, 26, 322, 28],
        [335, 29, 328, 26],
        [481, 33, 476, 27],
        [322, 28, 318, 33],
        [264, 49, 257, 47],
        [548, 50, 540, 47],
        [257, 47, 250, 51],
        [552, 56, 548, 50],
        [395, 20, 370, 57],
        [429, 57, 408, 23],
        [362, 58, 335, 29],
        [469, 26, 438, 58],
        [370, 57, 362, 58],
        [438, 58, 429, 57],
        [495, 69, 481, 33],
        [318, 33, 305, 69],
        [295, 71, 264, 49],
        [540, 47, 504, 71],
        [504, 71, 495, 69],
        [305, 69, 295, 71],
        [198, 82, 191, 82],
        [613, 85, 607, 81],
        [191, 82, 185, 87],
        [393, 87, 404, 87],
        [382, 93, 393, 87],
        [616, 92, 613, 85],
        [404, 87, 415, 94],
        [558, 94, 552, 56],
        [250, 51, 241, 94],
        [241, 94, 233, 98],
        [566, 98, 558, 94],
        [233, 98, 198, 82],
        [607, 81, 566, 98],
        [377, 103, 382, 93],
        [415, 94, 420, 104],
        [615, 107, 616, 92],
        [377, 115, 377, 103],
        [420, 104, 420, 115],
        [615, 120, 615, 107],
        [382, 125, 377, 115],
        [420, 115, 414, 125],
        [451, 124, 471, 128],
        [318, 131, 348, 124],
        [140, 128, 133, 129],
        [667, 130, 660, 128],
        [185, 87, 185, 131],
        [614, 131, 615, 120],
        [392, 131, 382, 125],
        [414, 125, 403, 131],
        [403, 131, 392, 131],
        [670, 134, 667, 130],
        [133, 129, 128, 134],
        [471, 128, 510, 141],
        [185, 131, 177, 136],
        [622, 136, 614, 131],
        [289, 142, 318, 131],
        [177, 136, 140, 128],
        [660, 128, 622, 136],
        [671, 140, 670, 134],
        [262, 155, 289, 142],
        [510, 141, 545, 160],
        [348, 124, 384, 162],
        [237, 172, 262, 155],
        [405, 165, 451, 124],
        [384, 162, 395, 167],
        [395, 167, 405, 165],
        [545, 160, 577, 183],
        [128, 134, 136, 177],
        [663, 177, 671, 140],
        [213, 191, 237, 172],
        [668, 185, 663, 177],
        [136, 177, 131, 185],
        [131, 185, 92, 183],
        [707, 183, 668, 185],
        [713, 186, 707, 183],
        [92, 183, 85, 186],
        [717, 191, 713, 186],
        [85, 186, 82, 191],
        [577, 183, 606, 210],
        [717, 198, 717, 191],
        [82, 191, 82, 198],
        [192, 211, 213, 191],
        [490, 211, 192, 211],
        [501, 213, 490, 211],
        [606, 210, 630, 242],
        [701, 233, 717, 198],
        [82, 198, 98, 233],
        [529, 222, 501, 213],
        [705, 241, 701, 233],
        [98, 233, 94, 241],
        [630, 242, 641, 259],
        [94, 241, 56, 248],
        [743, 248, 705, 241],
        [56, 248, 50, 250],
        [749, 250, 743, 248],
        [50, 250, 47, 257],
        [752, 258, 749, 250],
        [47, 257, 49, 264],
        [750, 264, 752, 258],
        [584, 271, 556, 238],
        [119, 293, 132, 291],
        [666, 292, 677, 294],
        [132, 291, 142, 295],
        [728, 295, 750, 264],
        [49, 264, 71, 295],
        [654, 296, 666, 292],
        [728, 296, 728, 295],
        [172, 296, 211, 296],
        [590, 305, 584, 271],
        [211, 296, 211, 474],
        [641, 259, 622, 304],
        [173, 299, 172, 296],
        [337, 298, 337, 350],
        [337, 298, 432, 298],
        [432, 298, 435, 298],
        [111, 301, 119, 293],
        [677, 294, 686, 302],
        [71, 295, 69, 301],
        [69, 301, 69, 302],
        [150, 305, 142, 295],
        [647, 305, 654, 296],
        [69, 302, 69, 305],
        [730, 305, 728, 296],
        [435, 298, 457, 308],
        [106, 311, 111, 301],
        [686, 302, 690, 312],
        [150, 305, 152, 316],
        [646, 317, 647, 305],
        [69, 305, 33, 318],
        [767, 318, 730, 305],
        [622, 304, 613, 322],
        [108, 323, 106, 311],
        [457, 308, 465, 325],
        [690, 312, 688, 324],
        [33, 318, 27, 323],
        [772, 324, 767, 318],
        [184, 326, 173, 299],
        [152, 316, 147, 326],
        [649, 328, 646, 317],
        [773, 329, 772, 324],
        [27, 323, 26, 330],
        [582, 337, 590, 305],
        [115, 332, 108, 323],
        [613, 322, 613, 333],
        [688, 324, 681, 333],
        [147, 326, 138, 334],
        [659, 335, 649, 328],
        [770, 335, 773, 329],
        [181, 337, 184, 326],
        [126, 336, 115, 332],
        [138, 334, 126, 336],
        [681, 333, 670, 337],
        [465, 325, 458, 340],
        [670, 337, 659, 335],
        [613, 333, 618, 342],
        [174, 344, 181, 337],
        [123, 367, 174, 344],
        [458, 340, 438, 348],
        [438, 348, 428, 350],
        [428, 350, 337, 350],
        [26, 330, 58, 362],
        [742, 362, 770, 335],
        [618, 342, 675, 369],
        [742, 369, 742, 362],
        [548, 374, 582, 337],
        [675, 369, 676, 369],
        [742, 370, 742, 369],
        [58, 362, 57, 370],
        [533, 383, 548, 374],
        [121, 387, 123, 367],
        [57, 370, 23, 391],
        [742, 370, 776, 391],
        [550, 397, 533, 383],
        [780, 396, 776, 391],
        [23, 391, 20, 397],
        [558, 405, 550, 397],
        [780, 402, 780, 396],
        [20, 397, 20, 404],
        [562, 410, 558, 405],
        [776, 408, 780, 402],
        [565, 416, 562, 410],
        [676, 369, 677, 418],
        [569, 422, 565, 416],
        [123, 423, 121, 387],
        [677, 418, 647, 418],
        [647, 418, 645, 420],
        [337, 425, 411, 425],
        [337, 476, 337, 425],
        [411, 425, 417, 426],
        [742, 429, 776, 408],
        [20, 404, 57, 429],
        [574, 438, 569, 422],
        [417, 426, 434, 434],
        [645, 420, 644, 442],
        [742, 438, 742, 429],
        [57, 429, 58, 438],
        [126, 449, 123, 423],
        [434, 434, 450, 456],
        [450, 456, 453, 463],
        [644, 442, 637, 463],
        [580, 462, 574, 438],
        [58, 438, 29, 464],
        [770, 464, 742, 438],
        [132, 474, 126, 449],
        [588, 470, 580, 462],
        [773, 471, 770, 464],
        [29, 464, 26, 471],
        [637, 463, 623, 473],
        [211, 474, 132, 474],
        [605, 475, 588, 470],
        [623, 473, 605, 475],
        [406, 476, 337, 476],
        [771, 477, 773, 471],
        [26, 471, 28, 478],
        [407, 478, 406, 476],
        [766, 481, 771, 477],
        [730, 495, 766, 481],
        [28, 478, 69, 495],
        [728, 504, 730, 495],
        [69, 495, 71, 504],
        [453, 463, 464, 515],
        [464, 515, 467, 527],
        [71, 504, 49, 535],
        [752, 540, 728, 504],
        [49, 535, 47, 541],
        [47, 541, 50, 548],
        [749, 548, 752, 540],
        [467, 527, 485, 552],
        [50, 548, 56, 552],
        [743, 552, 749, 548],
        [705, 558, 743, 552],
        [56, 552, 94, 558],
        [407, 560, 407, 478],
        [485, 552, 502, 562],
        [407, 561, 407, 560],
        [502, 562, 506, 562],
        [405, 562, 407, 561],
        [621, 562, 624, 562],
        [175, 562, 405, 562],
        [506, 562, 621, 562],
        [94, 558, 98, 566],
        [701, 566, 705, 558],
        [255, 581, 200, 590],
        [548, 581, 537, 583],
        [711, 588, 701, 566],
        [265, 585, 255, 581],
        [624, 562, 600, 590],
        [537, 583, 529, 590],
        [200, 590, 175, 562],
        [600, 590, 598, 592],
        [270, 593, 265, 585],
        [598, 592, 548, 581],
        [98, 566, 82, 601],
        [717, 601, 711, 588],
        [717, 609, 717, 601],
        [82, 601, 82, 609],
        [227, 609, 238, 610],
        [558, 611, 571, 610],
        [714, 613, 717, 609],
        [82, 609, 86, 613],
        [216, 615, 227, 609],
        [571, 610, 581, 616],
        [238, 610, 248, 617],
        [86, 613, 92, 616],
        [707, 616, 714, 613],
        [92, 616, 131, 614],
        [668, 614, 707, 616],
        [549, 618, 558, 611],
        [666, 617, 668, 614],
        [131, 614, 136, 622],
        [663, 622, 666, 617],
        [210, 625, 216, 615],
        [581, 616, 587, 626],
        [248, 617, 253, 628],
        [544, 628, 549, 618],
        [209, 637, 210, 625],
        [587, 626, 588, 638],
        [281, 641, 270, 593],
        [253, 628, 252, 639],
        [544, 640, 544, 628],
        [214, 647, 209, 637],
        [588, 638, 582, 648],
        [252, 639, 247, 648],
        [551, 649, 544, 640],
        [283, 649, 281, 641],
        [291, 653, 283, 649],
        [529, 590, 514, 650],
        [224, 653, 214, 647],
        [236, 654, 247, 648],
        [582, 648, 572, 654],
        [561, 655, 551, 649],
        [236, 654, 224, 653],
        [572, 654, 561, 655],
        [136, 622, 128, 659],
        [514, 650, 478, 664],
        [329, 666, 291, 653],
        [671, 665, 663, 622],
        [128, 659, 129, 666],
        [614, 668, 622, 663],
        [177, 663, 185, 668],
        [666, 670, 671, 665],
        [129, 666, 134, 671],
        [478, 664, 439, 672],
        [622, 663, 659, 671],
        [659, 671, 666, 670],
        [134, 671, 177, 663],
        [368, 673, 329, 666],
        [439, 672, 419, 674],
        [389, 675, 368, 673],
        [419, 674, 389, 675],
        [558, 705, 566, 701],
        [233, 701, 241, 705],
        [185, 668, 183, 707],
        [614, 712, 614, 668],
        [183, 707, 186, 713],
        [198, 717, 233, 701],
        [566, 701, 601, 717],
        [609, 717, 614, 712],
        [186, 713, 191, 718],
        [191, 718, 198, 717],
        [601, 717, 609, 717],
        [495, 730, 504, 728],
        [295, 728, 305, 730],
        [241, 705, 248, 743],
        [552, 743, 558, 705],
        [362, 742, 370, 742],
        [429, 742, 438, 742],
        [549, 749, 552, 743],
        [248, 743, 251, 749],
        [504, 728, 535, 750],
        [542, 752, 549, 749],
        [251, 749, 259, 752],
        [259, 752, 295, 728],
        [535, 750, 542, 752],
        [305, 730, 318, 766],
        [481, 766, 495, 730],
        [438, 742, 464, 770],
        [334, 771, 362, 742],
        [318, 766, 322, 771],
        [476, 772, 481, 766],
        [464, 770, 470, 773],
        [322, 771, 327, 773],
        [327, 773, 334, 771],
        [470, 773, 476, 772],
        [370, 742, 391, 776],
        [404, 779, 429, 742],
        [391, 776, 397, 780],
        [397, 780, 404, 779],
        [556, 238, 529, 222],
    ];
    let segments: Vec<Line<i32>> = segments.iter().map(|x| x.into()).collect();

    let mut centerline = Centerline::<i32, f32, i64, f64>::with_segments(segments);
    centerline.build_voronoi()?;
    println!(
        "Result: cells:{}, edges:{}, vertices:{}",
        centerline.diagram().cells().len(),
        centerline.diagram().edges().len(),
        centerline.diagram().vertices().len()
    );
    let _= centerline.calculate_centerline(0.38);
    println!(
        "Result: lines:{}, arcs:{}, linsestrings:{}",
        centerline.lines.len(),
        centerline.arcs.len(),
        centerline.linestrings.len()
    );
    b.iter(move || {
        let _ = centerline.calculate_centerline(0.38);
    });


    Ok(())
}
